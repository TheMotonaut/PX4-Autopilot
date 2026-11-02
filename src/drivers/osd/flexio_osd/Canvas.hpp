/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
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

/**
 * @file Canvas.hpp
 *
 * Drawing on a FlexIO OSD frame.
 */

#pragma once

#include <drivers/drv_flexio_osd.h>

class Canvas
{
public:
	static constexpr int WIDTH = FLEXIO_OSD_WIDTH;
	static constexpr int HEIGHT = FLEXIO_OSD_HEIGHT;

	// Character cell of draw_text(): the glyph plus one pixel of outline on each side
	static constexpr int CHAR_WIDTH = 6;
	static constexpr int CHAR_HEIGHT = 9;

	enum class Align {
		Left,
		Center,
		Right,
	};

	const flexio_osd_frame_s &frame() const { return _frame; }

	/**
	 * Make every pixel transparent
	 */
	void clear();

	/**
	 * Set one pixel, pixels outside the frame are ignored
	 */
	void set_pixel(int x, int y, flexio_osd_color_t color);

	void draw_hline(int x, int y, int length, flexio_osd_color_t color);
	void draw_vline(int x, int y, int length, flexio_osd_color_t color);

	/**
	 * Draw white text with a black outline, so it reads on bright and dark video.
	 *
	 * @param x	Left edge for Align::Left, center for Align::Center, right edge for Align::Right
	 * @param y	Top edge of the character cell
	 */
	void draw_text(int x, int y, const char *text, Align align = Align::Left);

	static int text_width(const char *text);

private:
	void draw_glyph(int x, int y, char c, bool outline);

	flexio_osd_frame_s _frame{};
};
