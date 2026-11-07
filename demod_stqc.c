/*
 *      demod_stqc.c -- STQC selective call demodulator
 *
 *      Copyright (C) 2013
 *          Jacek Lipkowski SQ5BPF (sq5bpf@lipkowski.org)
 *          Original STQC decoder, https://github.com/sq5bpf/multimon-ng-stqc
 *
 *      Copyright (C) 2026
 *          Wojciech Marmurowicz
 *
 *      Tone detection based upon selcall.c
 *      Copyright (C) 1996
 *          Thomas Sailer (sailer@ife.ee.ethz.ch, hb9jnx@hb9w.che.eu)
 *      Copyright (C) 2013
 *          Elias Oenal    (EliasOenal@gmail.com)
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

/*
 * STQC is a five-tone sequential selective call used by Digitex/Platan
 * equipment (e.g. DSP-50 siren controllers of Polish fire brigades).
 *
 * Tones 0-3 carry base-4 digits, most significant first. Tone 4 replaces
 * a digit equal to the previous one; the digit before the first tone is
 * assumed to be 0. Tones last 100 ms and follow each other without a pause.
 * A call usually consists of a 7-tone address, a 200 ms pause and an
 * 8-tone command; each block is reported as a separate sequence.
 */

#define SAMPLE_RATE 22050
#define PHINC(x) ((x)*0x10000/SAMPLE_RATE)

#include "multimon.h"
#include "filter.h"
#include <stdio.h>
#include <string.h>
#include "cJSON.h"

/* ---------------------------------------------------------------------- */

#define BLOCKLEN (SAMPLE_RATE/100)  /* 10 ms blocks */
#define BLOCKNUM 4                  /* must match l1_state_stqc in multimon.h */

/* Blocks a tone must persist before it is accepted; rejects transients
 * at tone transitions and short bursts of speech or music. */
#define MIN_TONE_BLOCKS 3
/* Blocks without a valid tone that terminate a sequence. Shorter than the
 * pause between address and command (nominally 200 ms, about 110 ms seen in
 * recordings), longer than tone transitions. */
#define GAP_BLOCKS 8
/* A tone longer than this cannot be part of an STQC sequence (nominal
 * 100 ms) and terminates the current one. Catches echo tails that bridge
 * the pause in overdriven audio, where LEVEL_DROP cannot separate them. */
#define MAX_TONE_BLOCKS 16
/* Tone energy relative to the strongest recent energy of the same tone
 * below which the tone is treated as absent. Suppresses room echo and radio
 * tails that would otherwise bridge the pause between address and command
 * or start the next sequence with a spurious tone. The reference is kept
 * per tone because receiver de-emphasis and audio filters attenuate the
 * tones unequally; differences of up to 18 dB between the tones of one call
 * were seen in recordings. */
#define LEVEL_DROP 0.1f             /* -10 dB */
/* Per-block decay of the reference levels while no sequence is active. */
#define PEAK_DECAY 0.9f
/* Dominant tone energy relative to every other tone (-7 dB). Lower than
 * the 10 dB used by selcall.c to tolerate echo and audio distortion. */
#define DOMINANCE 0.2f
/* Minimum share of the band-limited block energy carried by the dominant
 * tone. A weak tone following a strong one shares the correlation window
 * with the tail of the strong tone and reaches about 15 %. */
#define MIN_FRACTION 0.15f
/* A tone whose energy rises by this factor (+6 dB) above its lowest level
 * starts again: the part before the rise was a tail. Separates the first
 * command tone from a tail of the same tone that stays strong through the
 * pause in overdriven audio. The level is tracked from RISE_AFTER blocks on,
 * when the correlation window no longer overlaps the tone onset; within a
 * tone, levels varied by up to 4 dB in recordings. */
#define RISE 4.0f
#define RISE_AFTER 10
/* Shorter sequences are discarded as noise. */
#define MIN_SEQ_TONES 3

static const unsigned int stqc_freq[STQC_NUM_TONES] = {
    PHINC(980), PHINC(1197), PHINC(1446), PHINC(1795), PHINC(2105)
};

/* The block energy compared against the dominant tone is limited to the tone
 * range by a 4th-order Butterworth low-pass at 2500 Hz. Energy above the
 * tones (harmonics of distorted tones, hiss) would otherwise count against
 * them; a tone whose third harmonic carried 90 % of the energy was seen.
 * Two biquads {b0, a1, a2} with b1 = 2 b0 and b2 = b0, for 22050 Hz. */
static const float lpf_coef[2][3] = {
    { 0.075813542f, -0.943724376f, 0.246978545f },
    { 0.097266454f, -1.210769482f, 0.599835298f },
};

/* Transposed direct form II. */
static inline float biquad(const float *c, float *z, float x)
{
    float y = c[0] * x + z[0];

    z[0] = 2 * c[0] * x - c[1] * y + z[1];
    z[1] = c[0] * x - c[2] * y;
    return y;
}

extern int json_mode;

/* ---------------------------------------------------------------------- */

static void stqc_init(struct demod_state *s)
{
    memset(&s->l1.stqc, 0, sizeof(s->l1.stqc));
    s->l1.stqc.cand = -1;
    s->l1.stqc.lastch = -1;
}

/* Returns the dominant tone index or -1, and its energy in *tone_energy.
 * The tone given in prev is not compared against a different dominant tone:
 * its echo decays slowly in reverberant recordings and would otherwise mask
 * the following tone. */
static int process_block(struct demod_state *s, int prev, float *tone_energy)
{
    struct l1_state_stqc *st = &s->l1.stqc;
    float tote = 0;
    float totte[2 * STQC_NUM_TONES];
    float max_en = 0, limit;
    int i, j, idx = -1;

    for (i = 0; i < BLOCKNUM; i++)
        tote += st->energy[i];
    for (i = 0; i < 2 * STQC_NUM_TONES; i++) {
        totte[i] = 0;
        for (j = 0; j < BLOCKNUM; j++)
            totte[i] += st->tenergy[j][i];
    }
    for (i = 0; i < STQC_NUM_TONES; i++)
        totte[i] = fsqr(totte[i]) + fsqr(totte[i + STQC_NUM_TONES]);
    memmove(st->energy + 1, st->energy,
            sizeof(st->energy) - sizeof(st->energy[0]));
    st->energy[0] = 0;
    memmove(st->tenergy + 1, st->tenergy,
            sizeof(st->tenergy) - sizeof(st->tenergy[0]));
    memset(st->tenergy, 0, sizeof(st->tenergy[0]));
    tote *= (BLOCKNUM * BLOCKLEN * 0.5);  /* adjust for block lengths */
    verbprintf(10, "stqc: Energies: %8.5f  %8.5f %8.5f %8.5f %8.5f %8.5f\n",
               tote, totte[0], totte[1], totte[2], totte[3], totte[4]);

    for (i = 0; i < STQC_NUM_TONES; i++)
        if (totte[i] > max_en) {
            max_en = totte[i];
            idx = i;
        }
    if (idx < 0)
        return -1;
    limit = max_en * DOMINANCE;
    for (i = 0; i < STQC_NUM_TONES; i++)
        if (i != idx && i != prev && totte[i] > limit)
            return -1;
    if ((tote * MIN_FRACTION) > max_en)
        return -1;
    *tone_energy = max_en;
    return idx;
}

static void report_sequence(const struct l1_state_stqc *st)
{
    char digits[STQC_MAX_TONES + 1];
    char prev = '0';
    unsigned long long value = 0;
    unsigned int i;

    for (i = 0; i < st->ntones; i++) {
        if (st->tones[i] != '4')
            prev = st->tones[i];
        digits[i] = prev;
        value = value * 4 + (unsigned int)(prev - '0');
    }
    digits[i] = '\0';

    if (!json_mode) {
        /* Same layout as the original sq5bpf decoder, so existing parsers
         * keep working. */
        verbprintf(0, "STQC: %u tones  orig:[%s] parsed:[%s]  VALUES: dec: %llu hex: 0x%llX\n",
                   st->ntones, st->tones, digits, value, value);
    } else {
        cJSON *json_output = cJSON_CreateObject();
        cJSON_AddStringToObject(json_output, "demod_name", "STQC");
        cJSON_AddStringToObject(json_output, "tones", st->tones);
        cJSON_AddStringToObject(json_output, "digits", digits);
        /* Exact as a JSON number: STQC_MAX_TONES base-4 digits fit in 2^53. */
        cJSON_AddNumberToObject(json_output, "value", (double)value);
        addJsonTimestamp(json_output);
        printJson(json_output);
        cJSON_Delete(json_output);
    }
}

static void end_sequence(struct l1_state_stqc *st)
{
    if (st->ntones >= MIN_SEQ_TONES)
        report_sequence(st);
    st->ntones = 0;
    st->tones[0] = '\0';
}

static void process_tone(struct l1_state_stqc *st, int i, float en)
{
    int j;

    if (st->ntones == 0)
        for (j = 0; j < STQC_NUM_TONES; j++)
            st->peak[j] *= PEAK_DECAY;
    if (i >= 0 && en < st->peak[i] * LEVEL_DROP)
        i = -1;

    if (i >= 0 && i == st->lastch && st->tone_blocks >= RISE_AFTER) {
        if (st->tone_min == 0 || en < st->tone_min)
            st->tone_min = en;
        if (en > st->tone_min * RISE) {
            end_sequence(st);
            st->lastch = -1;
            st->cand = i;
            st->cand_blocks = 1;
            st->gap_blocks = 0;
            return;
        }
    }

    if (i >= 0 && i == st->cand) {
        st->cand_blocks++;
    } else {
        st->cand = i;
        st->cand_blocks = (i >= 0) ? 1 : 0;
    }

    if (i < 0) {
        /* lastch is cleared even without an active sequence: a tone that
         * ended a sequence by its length must not suppress the first tone of
         * the next sequence. The counters saturate for continuous operation. */
        if (st->gap_blocks < GAP_BLOCKS && ++st->gap_blocks == GAP_BLOCKS) {
            end_sequence(st);
            st->lastch = -1;
        }
        return;
    }

    st->gap_blocks = 0;
    if (i == st->lastch) {
        if (en > st->peak[i])
            st->peak[i] = en;
        /* lastch stays set until the tone stops or rises, so it is not
         * re-added. */
        if (st->tone_blocks <= MAX_TONE_BLOCKS && ++st->tone_blocks > MAX_TONE_BLOCKS)
            end_sequence(st);
        return;
    }
    if (st->cand_blocks < MIN_TONE_BLOCKS)
        return;

    if (st->ntones >= STQC_MAX_TONES)
        end_sequence(st);
    st->tones[st->ntones++] = (char)('0' + i);
    st->tones[st->ntones] = '\0';
    st->lastch = i;
    st->tone_blocks = st->cand_blocks;
    st->tone_min = 0;
    if (en > st->peak[i])
        st->peak[i] = en;
}

static void stqc_demod(struct demod_state *s, buffer_t buffer, int length)
{
    struct l1_state_stqc *st = &s->l1.stqc;
    const float *in = buffer.fbuffer;
    float s_in, s_lp, en = 0;
    int i;

    for (; length > 0; length--, in++) {
        s_in = *in;
        s_lp = biquad(lpf_coef[0], st->lpf[0], s_in);
        s_lp = biquad(lpf_coef[1], st->lpf[1], s_lp);
        st->energy[0] += fsqr(s_lp);
        for (i = 0; i < STQC_NUM_TONES; i++) {
            st->tenergy[0][i] += COS(st->ph[i]) * s_in;
            st->tenergy[0][i + STQC_NUM_TONES] += SIN(st->ph[i]) * s_in;
            st->ph[i] += stqc_freq[i];
        }
        if ((st->blkcount--) <= 0) {
            st->blkcount = BLOCKLEN;
            i = process_block(s, st->lastch, &en);
            process_tone(st, i, en);
        }
    }
}

static void stqc_deinit(struct demod_state *s)
{
    end_sequence(&s->l1.stqc);
}

const struct demod_param demod_stqc = {
    "STQC", true, SAMPLE_RATE, 0, stqc_init, stqc_demod, stqc_deinit
};
