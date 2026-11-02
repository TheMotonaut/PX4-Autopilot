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

#include "Canvas.hpp"
#include "font5x7.h"

#include <string.h>

void Canvas::clear()
{
	memset(&_frame, 0, sizeof(_frame));
}

void Canvas::set_pixel(int x, int y, flexio_osd_color_t color)
{
	if (x < 0 || x >= WIDTH || y < 0 || y >= HEIGHT) {
		return;
	}

	const int word = x / 32;
	const uint32_t bit = 1u << (x % 32);

	if (color == FLEXIO_OSD_TRANSPARENT) {
		_frame.enable[y][word] &= ~bit;

	} else {
		_frame.enable[y][word] |= bit;
	}

	if (color == FLEXIO_OSD_WHITE) {
		_frame.white[y][word] |= bit;

	} else {
		_frame.white[y][word] &= ~bit;
	}
}

void Canvas::draw_hline(int x, int y, int length, flexio_osd_color_t color)
{
	for (int i = 0; i < length; i++) {
		set_pixel(x + i, y, color);
	}
}

void Canvas::draw_vline(int x, int y, int length, flexio_osd_color_t color)
{
	for (int i = 0; i < length; i++) {
		set_pixel(x, y + i, color);
	}
}

int Canvas::text_width(const char *text)
{
	return static_cast<int>(strlen(text)) * CHAR_WIDTH;
}

void Canvas::draw_glyph(int x, int y, char c, bool outline)
{
	if (c < font5x7::FIRST_CHAR || c > font5x7::LAST_CHAR) {
		c = '?';
	}

	const uint8_t *glyph = font5x7::glyphs[c - font5x7::FIRST_CHAR];

	for (int column = 0; column < font5x7::WIDTH; column++) {
		for (int row = 0; row < font5x7::HEIGHT; row++) {
			if ((glyph[column] & (1u << row)) == 0) {
				continue;
			}

			if (outline) {
				for (int dy = -1; dy <= 1; dy++) {
					for (int dx = -1; dx <= 1; dx++) {
						set_pixel(x + column + dx, y + row + dy, FLEXIO_OSD_BLACK);
					}
				}

			} else {
				set_pixel(x + column, y + row, FLEXIO_OSD_WHITE);
			}
		}
	}
}

void Canvas::draw_text(int x, int y, const char *text, Align align)
{
	const int width = text_width(text);

	if (align == Align::Center) {
		x -= width / 2;

	} else if (align == Align::Right) {
		x -= width;
	}

	// The glyph starts inside the cell, after the outline pixel
	x += 1;
	y += 1;

	// Outline the whole string first so the outline of one glyph never covers its neighbour
	for (int pass = 0; pass < 2; pass++) {
		const bool outline = (pass == 0);
		int glyph_x = x;

		for (const char *c = text; *c != '\0'; c++) {
			draw_glyph(glyph_x, y, *c, outline);
			glyph_x += CHAR_WIDTH;
		}
	}
}
