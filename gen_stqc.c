/*
 *      gen_stqc.c -- generate STQC selective call sequences
 *
 *      Copyright (C) 2026
 *          Wojciech Marmurowicz
 *
 *      STQC protocol description by Jacek Lipkowski SQ5BPF,
 *      https://github.com/sq5bpf/multimon-ng-stqc
 *
 *      This program is free software; you can redistribute it and/or modify
 *      it under the terms of the GNU General Public License as published by
 *      the Free Software Foundation; either version 2 of the License, or
 *      (at your option) any later version.
 *
 *      This program is distributed in the hope that it will be useful,
 *      but WITHOUT ANY WARRANTY; without even the implied warranty of
 *      MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *      GNU General Public License for more details.
 *
 *      You should have received a copy of the GNU General Public License
 *      along with this program; if not, write to the Free Software
 *      Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

/* ---------------------------------------------------------------------- */

#include "gen.h"
#include <string.h>
#include <stdio.h>

/* ---------------------------------------------------------------------- */

#define PHINC(x)   ((float)(x)*0x10000/SAMPLE_RATE)

static const unsigned int stqc_freq[5] = {
	PHINC(980), PHINC(1197), PHINC(1446), PHINC(1795), PHINC(2105)
};

void gen_init_stqc(struct gen_params *p, struct gen_state *s)
{
	(void)p;
	memset(s, 0, sizeof(struct gen_state));
}

/*
 * Characters '0'-'4' select a tone, a space inserts a pause. Other
 * characters are reported and skipped.
 * Consecutive tones are phase-continuous and have no gap between them,
 * as transmitted by STQC encoders.
 */
int gen_stqc(signed short *buf, int buflen, struct gen_params *p, struct gen_state *s)
{
	char c;
	int num = 0;

	for (; buflen > 0; buflen--, buf++, num++) {
		while (s->s.stqc.time <= 0) {
			c = p->p.stqc.str[s->s.stqc.ch_idx];
			if (!c)
				return num;
			s->s.stqc.ch_idx++;
			if (c >= '0' && c <= '4') {
				s->s.stqc.time = p->p.stqc.duration;
				s->s.stqc.phinc = stqc_freq[c - '0'];
			} else if (c == ' ') {
				s->s.stqc.time = p->p.stqc.pause;
				s->s.stqc.phinc = 0;
				s->s.stqc.ph = 0xc000;
			} else {
				fprintf(stderr, "gen: stqc; invalid char '%c'\n", c);
			}
		}
		s->s.stqc.time--;
		if (s->s.stqc.phinc)
			*buf += (p->ampl * COS(s->s.stqc.ph)) >> 15;
		s->s.stqc.ph += s->s.stqc.phinc;
	}
	return num;
}
