/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/*
 * FlexIO resources, all on one instance:
 *
 *  Timer 1 (delay)   started by the HSYNC rising edge, counts from the sync edge to the first overlay pixel.
 *  Timer 0 (pixel)   started when the delay timer expires, clocks 128 pixels and a low stop bit.
 *  Timer 2 (vsync)   runs while HSYNC is high and only expires on the broad pulses of the vertical sync.
 *  Shifters 0-3      chained into one 128-bit transmitter on OSD_EN, shifter 0 drives the pin.
 *  Shifters 4-7      chained into one 128-bit transmitter on OSD_W, shifter 4 drives the pin.
 *
 * All eight shifters load their buffers when the pixel timer starts a line. That load raises the status flag of
 * shifter 0, and its interrupt queues the next line, leaving a whole line period to do so. The vsync timer
 * interrupt resets the line count and swaps in a new frame, so frames change only between fields.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/micro_hal.h>
#include <px4_platform_common/log.h>
#include <drivers/drv_flexio_osd.h>
#include <drivers/drv_hrt.h>
#include <board_config.h>
#include <errno.h>

#if defined(FLEXIO_OSD_HSYNC_PIN) && defined(FLEXIO_OSD_EN_PIN) && defined(FLEXIO_OSD_W_PIN)

#include <imxrt_flexio.h>
#include <hardware/imxrt_flexio.h>
#include <imxrt_periphclks.h>
#include <nuttx/irq.h>
#include <string.h>

#include "arm_internal.h"

#ifndef FLEXIO_OSD_INSTANCE
#define FLEXIO_OSD_INSTANCE		2
#endif

#if FLEXIO_OSD_INSTANCE == 1
#  if defined(CONFIG_DRIVERS_DSHOT)
#    error "DShot resets and owns FlexIO1, put the OSD on FlexIO2"
#  endif
#  define FLEXIO_BASE			IMXRT_FLEXIO1_BASE
#  define FLEXIO_IRQ			IMXRT_IRQ_FLEXIO1
#  define flexio_clock_enable()		imxrt_clockall_flexio1()
#  if !defined(BOARD_FLEXIO_OSD_FREQ)
#    define BOARD_FLEXIO_OSD_FREQ	BOARD_FLEXIO_PREQ
#  endif
#elif FLEXIO_OSD_INSTANCE == 2
#  define FLEXIO_BASE			IMXRT_FLEXIO2_BASE
#  define FLEXIO_IRQ			IMXRT_IRQ_FLEXIO2
#  define flexio_clock_enable()		imxrt_clockall_flexio2()
#else
#  error "FLEXIO_OSD_INSTANCE must be 1 or 2"
#endif

#if !defined(BOARD_FLEXIO_OSD_FREQ)
#  error "Board must define BOARD_FLEXIO_OSD_FREQ, the FlexIO clock of FLEXIO_OSD_INSTANCE"
#endif

#if defined(IOMUX_PULL_DOWN_100K)
#define IOMUX_PULL_DOWN IOMUX_PULL_DOWN_100K
#endif

#define PIXEL_TIMER			0u
#define DELAY_TIMER			1u
#define VSYNC_TIMER			2u

#define TIMERS_USED			3u

#define SHIFTERS_PER_OUTPUT		4u
#define EN_SHIFTER			0u
#define W_SHIFTER			(EN_SHIFTER + SHIFTERS_PER_OUTPUT)
#define SHIFTERS_USED			(2u * SHIFTERS_PER_OUTPUT)

#define NS_PER_S			1000000000ull

// Normal and equalizing sync pulses are at most 4.7 us, the broad vertical sync pulses about 27 us
#define VSYNC_PULSE_MIN_NS		10000u

// Broad pulses repeat every half line during vertical sync, a field is at least this long
#define FIELD_LINES_MIN			200u

// Between NTSC with 262.5 and PAL with 312.5 lines per field
#define PAL_FIELD_LINES_MIN		287u

// No vertical sync for this long means there is no video
#define VIDEO_TIMEOUT_US		100000u

// The dual 8-bit baud mode counts shift edges in 8 bits, a line has to fit one 128-bit transfer
#define PIXEL_TIMER_MAX_EDGES		256u

static_assert(FLEXIO_OSD_WORDS_PER_ROW == SHIFTERS_PER_OUTPUT, "one shifter per 32 pixels of a line");
static_assert(FLEXIO_OSD_WIDTH * 2 <= PIXEL_TIMER_MAX_EDGES, "line does not fit one pixel timer transfer");

typedef struct {
	uint32_t active_start_ns;	// HSYNC leading edge to the start of active video
	uint32_t active_width_ns;	// duration of active video
	uint16_t first_line;		// first visible line counted from the end of the vertical sync
	uint16_t visible_lines;		// visible lines per field
} video_timing_t;

static const video_timing_t video_timing[] = {
	[FLEXIO_OSD_STANDARD_PAL]  = { .active_start_ns = 10500, .active_width_ns = 52000, .first_line = 20, .visible_lines = 288 },
	[FLEXIO_OSD_STANDARD_NTSC] = { .active_start_ns = 9400,  .active_width_ns = 52600, .first_line = 15, .visible_lines = 240 },
};

static struct flexio_osd_frame_s frame_buffer[2];
static volatile uint8_t shown_frame;
static volatile int8_t pending_frame = -1;

static bool _initialized = false;
static flexio_osd_standard_t _standard;
static uint32_t _first_line;
static uint32_t _visible_lines;
static uint32_t _pixel_ns;

static volatile uint32_t line;			// line being shifted out, counted from the end of the vertical sync
static volatile uint32_t field_lines;		// lines in the last complete field
static volatile uint32_t field_count;
static volatile uint32_t underrun_count;
static volatile hrt_abstime last_vsync;

static inline uint32_t flexio_getreg32(uint32_t offset)
{
	return getreg32(FLEXIO_BASE + offset);
}

static inline void flexio_modifyreg32(unsigned int offset,
				      uint32_t clearbits,
				      uint32_t setbits)
{
	modifyreg32(FLEXIO_BASE + offset, clearbits, setbits);
}

static inline void flexio_putreg32(uint32_t value, uint32_t offset)
{
	putreg32(value, FLEXIO_BASE + offset);
}

static inline uint32_t ns_to_counts(uint64_t ns)
{
	return (uint32_t)((ns * BOARD_FLEXIO_OSD_FREQ + NS_PER_S / 2) / NS_PER_S);
}

// Write the pixels of one line into the shifter buffers, they go out on the next HSYNC
static inline __attribute__((always_inline)) void queue_line(uint32_t next_line)
{
	const uint32_t *enable = NULL;
	const uint32_t *white = NULL;

	if (next_line >= _first_line && next_line < _first_line + _visible_lines) {
		uint32_t row = (next_line - _first_line) * FLEXIO_OSD_HEIGHT / _visible_lines;
		const struct flexio_osd_frame_s *frame = &frame_buffer[shown_frame];
		enable = frame->enable[row];
		white = frame->white[row];
	}

	for (uint32_t word = 0; word < SHIFTERS_PER_OUTPUT; word++) {
		flexio_putreg32(enable ? enable[word] : 0, IMXRT_FLEXIO_SHIFTBUF0_OFFSET + (EN_SHIFTER + word) * 0x4);
		flexio_putreg32(white ? white[word] : 0, IMXRT_FLEXIO_SHIFTBUF0_OFFSET + (W_SHIFTER + word) * 0x4);
	}
}

static int flexio_osd_irq_handler(int irq, void *context, void *arg)
{
	if (flexio_getreg32(IMXRT_FLEXIO_TIMSTAT_OFFSET) & (1u << VSYNC_TIMER)) {
		flexio_putreg32(1u << VSYNC_TIMER, IMXRT_FLEXIO_TIMSTAT_OFFSET);

		// First broad pulse of this vertical sync
		if (line >= FIELD_LINES_MIN) {
			field_lines = line;
			field_count++;
			last_vsync = hrt_absolute_time();

			if (pending_frame >= 0) {
				shown_frame = pending_frame;
				pending_frame = -1;
			}
		}

		line = 0;
	}

	if (flexio_getreg32(IMXRT_FLEXIO_SHIFTSTAT_OFFSET) & (1u << EN_SHIFTER)) {
		// The buffers were not refilled in time and the line just started repeats the previous one
		uint32_t errors = flexio_getreg32(IMXRT_FLEXIO_SHIFTERR_OFFSET);

		if (errors) {
			flexio_putreg32(errors, IMXRT_FLEXIO_SHIFTERR_OFFSET);
			underrun_count++;
		}

		line++;
		queue_line(line + 1);
	}

	return OK;
}

static void configure_shifters(uint32_t first_shifter, uint32_t pin)
{
	for (uint32_t i = 0; i < SHIFTERS_PER_OUTPUT; i++) {
		uint32_t shifter = first_shifter + i;
		bool drives_pin = (i == 0);
		bool last_in_chain = (i == SHIFTERS_PER_OUTPUT - 1);

		flexio_putreg32(0, IMXRT_FLEXIO_SHIFTCTL0_OFFSET + shifter * 0x4);

		/* Each shifter takes its successor's data so the chain shifts out 128 bits. The stop bit
		 * leaves OSD_EN low, transparent, until the next line. */
		flexio_putreg32(FLEXIO_SHIFTCFG_INSRC(last_in_chain ? FLEXIO_SHIFTER_INPUT_FROM_PIN :
						      FLEXIO_SHIFTER_INPUT_FROM_NEXT_SHIFTER_OUTPUT) |
				FLEXIO_SHIFTCFG_PWIDTH(0) |
				FLEXIO_SHIFTCFG_SSTOP(FLEXIO_SHIFTER_STOP_BIT_LOW) |
				FLEXIO_SHIFTCFG_SSTART(FLEXIO_SHIFTER_START_BIT_DISABLED_LOAD_DATA_ON_ENABLE),
				IMXRT_FLEXIO_SHIFTCFG0_OFFSET + shifter * 0x4);

		flexio_putreg32(FLEXIO_SHIFTCTL_TIMSEL(PIXEL_TIMER) |
				FLEXIO_SHIFTCTL_TIMPOL(FLEXIO_SHIFTER_TIMER_POLARITY_ON_POSITIVE) |
				FLEXIO_SHIFTCTL_PINCFG(drives_pin ? FLEXIO_PIN_CONFIG_OUTPUT : FLEXIO_PIN_CONFIG_OUTPUT_DISABLED) |
				FLEXIO_SHIFTCTL_PINSEL(drives_pin ? pin : 0) |
				FLEXIO_SHIFTCTL_PINPOL(FLEXIO_PIN_ACTIVE_HIGH) |
				FLEXIO_SHIFTCTL_SMOD(FLEXIO_SHIFTER_MODE_TRANSMIT),
				IMXRT_FLEXIO_SHIFTCTL0_OFFSET + shifter * 0x4);
	}
}

static void configure_timers(uint32_t delay_counts, uint32_t pixel_half_counts)
{
	/* Delay: one shot from the HSYNC rising edge, its output falls when it expires */
	flexio_putreg32(0, IMXRT_FLEXIO_TIMCTL0_OFFSET + DELAY_TIMER * 0x4);

	flexio_putreg32(FLEXIO_TIMCFG_TIMOUT(FLEXIO_TIMER_OUTPUT_ONE_NOT_AFFECTED_BY_RESET) |
			FLEXIO_TIMCFG_TIMDEC(FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_SHIFT_TIMER_OUTPUT) |
			FLEXIO_TIMCFG_TIMRST(FLEXIO_TIMER_RESET_NEVER) |
			FLEXIO_TIMCFG_TIMDIS(FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE) |
			FLEXIO_TIMCFG_TIMENA(FLEXIO_TIMER_ENABLE_ON_TRIGGER_RISING_EDGE) |
			FLEXIO_TIMCFG_TSTOP(FLEXIO_TIMER_STOP_BIT_DISABLED) |
			FLEXIO_TIMCFG_TSTART(FLEXIO_TIMER_START_BIT_DISABLED),
			IMXRT_FLEXIO_TIMCFG0_OFFSET + DELAY_TIMER * 0x4);

	flexio_putreg32(delay_counts - 1, IMXRT_FLEXIO_TIMCMP0_OFFSET + DELAY_TIMER * 0x4);

	flexio_putreg32(FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_PININPUT(FLEXIO_OSD_HSYNC_PIN)) |
			FLEXIO_TIMCTL_TRGPOL(FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_HIGH) |
			FLEXIO_TIMCTL_TRGSRC(FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL) |
			FLEXIO_TIMCTL_PINCFG(FLEXIO_PIN_CONFIG_OUTPUT_DISABLED) |
			FLEXIO_TIMCTL_PINSEL(0) |
			FLEXIO_TIMCTL_PINPOL(FLEXIO_PIN_ACTIVE_HIGH) |
			FLEXIO_TIMCTL_TIMOD(FLEXIO_TIMER_MODE_SINGLE16_BIT),
			IMXRT_FLEXIO_TIMCTL0_OFFSET + DELAY_TIMER * 0x4);

	/* Pixel clock: triggered by the falling delay timer output, one line of pixels then a stop bit */
	flexio_putreg32(0, IMXRT_FLEXIO_TIMCTL0_OFFSET + PIXEL_TIMER * 0x4);

	flexio_putreg32(FLEXIO_TIMCFG_TIMOUT(FLEXIO_TIMER_OUTPUT_ONE_NOT_AFFECTED_BY_RESET) |
			FLEXIO_TIMCFG_TIMDEC(FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_SHIFT_TIMER_OUTPUT) |
			FLEXIO_TIMCFG_TIMRST(FLEXIO_TIMER_RESET_NEVER) |
			FLEXIO_TIMCFG_TIMDIS(FLEXIO_TIMER_DISABLE_ON_TIMER_COMPARE) |
			FLEXIO_TIMCFG_TIMENA(FLEXIO_TIMER_ENABLE_ON_TRIGGER_RISING_EDGE) |
			FLEXIO_TIMCFG_TSTOP(FLEXIO_TIMER_STOP_BIT_ENABLE_ON_TIMER_DISABLE) |
			FLEXIO_TIMCFG_TSTART(FLEXIO_TIMER_START_BIT_DISABLED),
			IMXRT_FLEXIO_TIMCFG0_OFFSET + PIXEL_TIMER * 0x4);

	flexio_putreg32(((FLEXIO_OSD_WIDTH * 2 - 1) << 8) | (pixel_half_counts - 1),
			IMXRT_FLEXIO_TIMCMP0_OFFSET + PIXEL_TIMER * 0x4);

	flexio_putreg32(FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_TIMn(DELAY_TIMER)) |
			FLEXIO_TIMCTL_TRGPOL(FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_LOW) |
			FLEXIO_TIMCTL_TRGSRC(FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL) |
			FLEXIO_TIMCTL_PINCFG(FLEXIO_PIN_CONFIG_OUTPUT_DISABLED) |
			FLEXIO_TIMCTL_PINSEL(0) |
			FLEXIO_TIMCTL_PINPOL(FLEXIO_PIN_ACTIVE_HIGH) |
			FLEXIO_TIMCTL_TIMOD(FLEXIO_TIMER_MODE_DUAL8_BIT_BAUD_BIT),
			IMXRT_FLEXIO_TIMCTL0_OFFSET + PIXEL_TIMER * 0x4);

	/* Vsync: counts while HSYNC is high, so only a broad pulse lets it expire and raise its flag */
	flexio_putreg32(0, IMXRT_FLEXIO_TIMCTL0_OFFSET + VSYNC_TIMER * 0x4);

	flexio_putreg32(FLEXIO_TIMCFG_TIMOUT(FLEXIO_TIMER_OUTPUT_ONE_NOT_AFFECTED_BY_RESET) |
			FLEXIO_TIMCFG_TIMDEC(FLEXIO_TIMER_DEC_SRC_ON_FLEX_IO_CLOCK_SHIFT_TIMER_OUTPUT) |
			FLEXIO_TIMCFG_TIMRST(FLEXIO_TIMER_RESET_NEVER) |
			FLEXIO_TIMCFG_TIMDIS(FLEXIO_TIMER_DISABLE_ON_TRIGGER_FALLING_EDGE) |
			FLEXIO_TIMCFG_TIMENA(FLEXIO_TIMER_ENABLE_ON_TRIGGER_RISING_EDGE) |
			FLEXIO_TIMCFG_TSTOP(FLEXIO_TIMER_STOP_BIT_DISABLED) |
			FLEXIO_TIMCFG_TSTART(FLEXIO_TIMER_START_BIT_DISABLED),
			IMXRT_FLEXIO_TIMCFG0_OFFSET + VSYNC_TIMER * 0x4);

	flexio_putreg32(ns_to_counts(VSYNC_PULSE_MIN_NS) - 1, IMXRT_FLEXIO_TIMCMP0_OFFSET + VSYNC_TIMER * 0x4);

	flexio_putreg32(FLEXIO_TIMCTL_TRGSEL(FLEXIO_TIMER_TRIGGER_SEL_PININPUT(FLEXIO_OSD_HSYNC_PIN)) |
			FLEXIO_TIMCTL_TRGPOL(FLEXIO_TIMER_TRIGGER_POLARITY_ACTIVE_HIGH) |
			FLEXIO_TIMCTL_TRGSRC(FLEXIO_TIMER_TRIGGER_SOURCE_INTERNAL) |
			FLEXIO_TIMCTL_PINCFG(FLEXIO_PIN_CONFIG_OUTPUT_DISABLED) |
			FLEXIO_TIMCTL_PINSEL(0) |
			FLEXIO_TIMCTL_PINPOL(FLEXIO_PIN_ACTIVE_HIGH) |
			FLEXIO_TIMCTL_TIMOD(FLEXIO_TIMER_MODE_SINGLE16_BIT),
			IMXRT_FLEXIO_TIMCTL0_OFFSET + VSYNC_TIMER * 0x4);
}

int up_flexio_osd_init(flexio_osd_standard_t standard, int32_t h_offset_ns, int32_t v_offset_lines)
{
	if (standard != FLEXIO_OSD_STANDARD_PAL && standard != FLEXIO_OSD_STANDARD_NTSC) {
		return -EINVAL;
	}

	const video_timing_t *timing = &video_timing[standard];

	// Widest pixel that still fits the line into active video, centered in it
	uint32_t pixel_half_counts = (uint32_t)(((uint64_t)timing->active_width_ns * BOARD_FLEXIO_OSD_FREQ) /
						(2ull * FLEXIO_OSD_WIDTH * NS_PER_S));

	if (pixel_half_counts == 0 || pixel_half_counts > UINT8_MAX + 1) {
		PX4_ERR("FlexIO clock %u Hz cannot time %u ns lines", (unsigned)BOARD_FLEXIO_OSD_FREQ,
			(unsigned)timing->active_width_ns);
		return -EINVAL;
	}

	uint32_t pixel_ns = (uint32_t)((2ull * pixel_half_counts * NS_PER_S) / BOARD_FLEXIO_OSD_FREQ);
	int64_t start_ns = (int64_t)timing->active_start_ns + (timing->active_width_ns - pixel_ns * FLEXIO_OSD_WIDTH) / 2
			   + h_offset_ns;
	uint32_t delay_counts = start_ns > 0 ? ns_to_counts((uint64_t)start_ns) : 0;

	if (delay_counts == 0 || delay_counts > UINT16_MAX + 1) {
		PX4_ERR("horizontal offset %li ns out of range", (long)h_offset_ns);
		return -EINVAL;
	}

	int32_t first_line = (int32_t)timing->first_line + v_offset_lines;

	if (first_line < 0) {
		PX4_ERR("vertical offset %li lines out of range", (long)v_offset_lines);
		return -EINVAL;
	}

	up_flexio_osd_deinit();

	flexio_clock_enable();

	uint32_t param = flexio_getreg32(IMXRT_FLEXIO_PARAM_OFFSET);

	if (((param & FLEXIO_PARAM_SHIFTER_MASK) >> FLEXIO_PARAM_SHIFTER_SHIFT) < SHIFTERS_USED ||
	    ((param & FLEXIO_PARAM_TIMER_MASK) >> FLEXIO_PARAM_TIMER_SHIFT) < TIMERS_USED) {
		PX4_ERR("FlexIO%d needs %u shifters and %u timers", FLEXIO_OSD_INSTANCE, SHIFTERS_USED, TIMERS_USED);
		return -ENODEV;
	}

	/* Reset FlexIO peripheral, before the driver state so a stale IRQ has nothing to service */
	flexio_modifyreg32(IMXRT_FLEXIO_CTRL_OFFSET, 0, FLEXIO_CTRL_SWRST_MASK);
	flexio_putreg32(0, IMXRT_FLEXIO_CTRL_OFFSET);

	memset(frame_buffer, 0, sizeof(frame_buffer));
	shown_frame = 0;
	pending_frame = -1;
	line = 0;
	field_lines = 0;
	field_count = 0;
	underrun_count = 0;
	last_vsync = 0;

	_standard = standard;
	_first_line = (uint32_t)first_line;
	_visible_lines = timing->visible_lines;
	_pixel_ns = pixel_ns;

	flexio_modifyreg32(IMXRT_FLEXIO_CTRL_OFFSET,
			   (FLEXIO_CTRL_DOZEN_MASK |
			    FLEXIO_CTRL_DBGE_MASK |
			    FLEXIO_CTRL_FASTACC_MASK |
			    FLEXIO_CTRL_FLEXEN_MASK),
			   (FLEXIO_CTRL_DBGE(1) |
			    FLEXIO_CTRL_FLEXEN(0)));

	// OSD_EN is pulled down so the overlay stays transparent whenever FlexIO does not drive it
	imxrt_config_gpio(FLEXIO_OSD_HSYNC_PINMUX);
	imxrt_config_gpio(FLEXIO_OSD_EN_PINMUX | IOMUX_PULL_DOWN);
	imxrt_config_gpio(FLEXIO_OSD_W_PINMUX);

	configure_shifters(EN_SHIFTER, FLEXIO_OSD_EN_PIN);
	configure_shifters(W_SHIFTER, FLEXIO_OSD_W_PIN);
	configure_timers(delay_counts, pixel_half_counts);

	// The first line is transparent, filling the buffers also clears their status flags
	queue_line(0);
	flexio_putreg32(UINT32_MAX, IMXRT_FLEXIO_SHIFTERR_OFFSET);
	flexio_putreg32(UINT32_MAX, IMXRT_FLEXIO_TIMSTAT_OFFSET);

	irq_attach(FLEXIO_IRQ, flexio_osd_irq_handler, 0);
	flexio_putreg32(1u << EN_SHIFTER, IMXRT_FLEXIO_SHIFTSIEN_OFFSET);
	flexio_putreg32(1u << VSYNC_TIMER, IMXRT_FLEXIO_TIMIEN_OFFSET);
	up_enable_irq(FLEXIO_IRQ);

	flexio_modifyreg32(IMXRT_FLEXIO_CTRL_OFFSET, 0, FLEXIO_CTRL_FLEXEN_MASK);

	_initialized = true;

	return OK;
}

void up_flexio_osd_deinit(void)
{
	if (!_initialized) {
		return;
	}

	_initialized = false;

	up_disable_irq(FLEXIO_IRQ);
	irq_detach(FLEXIO_IRQ);

	flexio_putreg32(0, IMXRT_FLEXIO_SHIFTSIEN_OFFSET);
	flexio_putreg32(0, IMXRT_FLEXIO_TIMIEN_OFFSET);

	// Released outputs fall back to the OSD_EN pull-down, transparent
	flexio_putreg32(0, IMXRT_FLEXIO_CTRL_OFFSET);
}

void up_flexio_osd_update(const struct flexio_osd_frame_s *frame)
{
	if (!_initialized) {
		return;
	}

	// Withdraw a frame not shown yet, so the vsync interrupt cannot swap to the buffer while it is written
	irqstate_t flags = px4_enter_critical_section();
	uint8_t back = shown_frame ^ 1u;
	pending_frame = -1;
	px4_leave_critical_section(flags);

	memcpy(&frame_buffer[back], frame, sizeof(frame_buffer[back]));

	flags = px4_enter_critical_section();
	pending_frame = (int8_t)back;
	px4_leave_critical_section(flags);
}

int up_flexio_osd_detected_standard(void)
{
	if (!_initialized) {
		return -1;
	}

	irqstate_t flags = px4_enter_critical_section();
	hrt_abstime vsync = last_vsync;
	uint32_t lines = field_lines;
	px4_leave_critical_section(flags);

	if (vsync == 0 || hrt_elapsed_time(&vsync) > VIDEO_TIMEOUT_US) {
		return -1;
	}

	return lines >= PAL_FIELD_LINES_MIN ? FLEXIO_OSD_STANDARD_PAL : FLEXIO_OSD_STANDARD_NTSC;
}

void up_flexio_osd_status(void)
{
	if (!_initialized) {
		PX4_INFO("not initialized");
		return;
	}

	int detected = up_flexio_osd_detected_standard();

	PX4_INFO("FlexIO%d, %s timing, %lu ns pixels", FLEXIO_OSD_INSTANCE,
		 _standard == FLEXIO_OSD_STANDARD_PAL ? "PAL" : "NTSC", (unsigned long)_pixel_ns);
	PX4_INFO("video: %s, %lu lines per field", detected < 0 ? "none" : (detected == FLEXIO_OSD_STANDARD_PAL ? "PAL" : "NTSC"),
		 (unsigned long)field_lines);
	PX4_INFO("fields: %lu, line underruns: %lu", (unsigned long)field_count, (unsigned long)underrun_count);
}

#else

int up_flexio_osd_init(flexio_osd_standard_t standard, int32_t h_offset_ns, int32_t v_offset_lines)
{
	return -ENODEV;
}

void up_flexio_osd_deinit(void)
{
}

void up_flexio_osd_update(const struct flexio_osd_frame_s *frame)
{
}

int up_flexio_osd_detected_standard(void)
{
	return -1;
}

void up_flexio_osd_status(void)
{
	PX4_INFO("board has no FlexIO OSD pins");
}

#endif
