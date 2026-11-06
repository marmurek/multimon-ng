# STQC

STQC is a five-tone sequential selective call used by Digitex/Platan equipment,
most notably the DSP-50 and DSP-52 controllers that activate sirens of Polish
volunteer fire brigades (OSP). The protocol description and the first decoder
come from Jacek Lipkowski SQ5BPF:
[sq5bpf/multimon-ng-stqc](https://github.com/sq5bpf/multimon-ng-stqc).

## Protocol

### Tones

| Tone | 0   | 1    | 2    | 3    | 4 (repeat) |
|------|-----|------|------|------|------------|
| Hz   | 980 | 1197 | 1446 | 1795 | 2105       |

Each tone lasts 100 ms. Tones within a sequence follow each other without a
pause, so a receiver can only detect a new symbol as a change of frequency.

### Digits and the repeat tone

A sequence carries a number in base 4, most significant digit first, padded
with leading zeros to the sequence length. Tones 0-3 carry the digits 0-3.

Two equal consecutive digits would produce one tone of double length, which
cannot be told apart from a single digit. Therefore a tone that would equal
the previously transmitted tone is replaced by tone 4 (2105 Hz). Two
consecutive tones are never equal.

The encoder starts as if tone 0 had already been transmitted. Because of the
zero padding most sequences begin with digit 0, which is then sent as tone 4.

The comparison is made against the transmitted tone, not the previous digit:
digits `000` become tones `404`, not `444`.

### Encoding example

Value 2075 as a 7-tone sequence:

1. Base 4, padded to 7 digits:
   2075 = 2·4⁵ + 0·4⁴ + 0·4³ + 1·4² + 2·4¹ + 3·4⁰ → `0200123`.
2. Replace digits equal to the previously transmitted tone:

   | Digit | Previous tone   | Transmitted tone |
   |-------|-----------------|------------------|
   | 0     | 0 (initial)     | 4                |
   | 2     | 4               | 2                |
   | 0     | 2               | 0                |
   | 0     | 0               | 4                |
   | 1     | 4               | 1                |
   | 2     | 1               | 2                |
   | 3     | 2               | 3                |

Result: tones `4204123`, i.e. 2105, 1446, 980, 2105, 1197, 1446, 1795 Hz.

### Decoding

1. Replace every tone 4 with the previous digit (0 at the start of the
   sequence): `4204123` → `0200123`.
2. Convert the digits from base 4: `0200123` → 2075.

The decoder reports both steps: `orig` holds the received tones, `parsed` the
digits and `dec` the value.

### Call structure

A DSP-50 call consists of two sequences separated by a pause of about 200 ms
(1.7 s in total):

    4204123   <200 ms>   40324202
    address, 7 tones     command, 8 tones
    value 2075           value 3746

- The 7-tone sequence selects the receiver (4⁷ = 16384 values).
- The 8-tone sequence carries the command (4⁸ = 65536 values).
- Controllers may answer with a short sequence (4 tones were observed, e.g.
  `0231`); its meaning is unknown.
- In recordings the pause between address and command varied from 110 to
  540 ms, and tone durations deviated from 100 ms because of echo.

The decoder reports each sequence separately and does not pair addresses with
commands.

### Meaning of values

The assignment of values to receivers and commands is not published by the
manufacturer. Community decoders interpret address/command pairs, e.g. the
STQC decoder by Bartek (serwerbartka.pl) in formats v2.10, v2.15 and v2.16,
documented in [airkey037/stqc_docs](https://github.com/airkey037/stqc_docs).
In format v2.16 the pair 2075/3746 above is interpreted as area 3227, unit
746, command `TEST`. Known command names include KASUJ, ALARM, PAGER, TEST,
MAKRO, OC.GR, AI.OC1-3 and FONIA. Such interpretation is outside the scope of
this decoder.

## Usage

    multimon-ng -a STQC -t raw -
    gen-ng -t wav -Q "4204123 40324202" call.wav

Each sequence is reported separately:

    STQC: 7 tones  orig:[4204123] parsed:[0200123]  VALUES: dec: 2075 hex: 0x81B

With `--json`:

    {"demod_name":"STQC","tones":"4204123","digits":"0200123","value":2075}

`orig`/`tones` is the received tone sequence, `parsed`/`digits` the sequence
with repeat tones resolved, `dec`/`value` its base-4 value. The text format
matches the original SQ5BPF decoder. Sequences shorter than 3 tones are
discarded as noise; sequences longer than 24 tones are split.

## Detection

Tone detection follows `selcall.c` (10 ms blocks, 40 ms correlation window).
The parameters cover radio audio with noise, loudspeaker audio recorded in
reverberant rooms and overdriven audio.

- A tone must dominate the other tones by 7 dB and carry 30 % of the block
  energy. The previous tone is excluded from the dominance check because its
  echo decays slowly.
- A tone must persist for 30 ms before it is accepted.
- A tone more than 12 dB below the strongest recent tone is ignored. This
  removes echo tails that would otherwise bridge the address/command pause or
  start the next sequence with a spurious tone.
- 80 ms without a valid tone, or a tone longer than 160 ms, ends a sequence.
  The pause between address and command was measured between 110 and 540 ms
  in recordings.
- A tone that ended a sequence by its length is ignored until it stops.
  If an echo tail stays within 12 dB for the whole pause and the address
  ends with the same tone as the command begins (tone 4), the first command
  tone merges with the tail and is lost; the value is unchanged because the
  lost tone is a leading zero. This affects acoustic captures (a loudspeaker
  recorded in a room); audio taken directly from a receiver output carries no
  such echo.

## Verification

### Independent encoders

Sequences produced by three independent encoders decode identically:

| Encoder | Output |
|---|---|
| `gen-ng -Q` (this repository) | raw 22050 Hz |
| `stqc.sh` by SQ5BPF ([multimon-ng-stqc](https://github.com/sq5bpf/multimon-ng-stqc)) | WAV 8 kHz, including the protocol example 1231 → `40103034` |
| `pystqc.py` ([airkey037/stqc_docs](https://github.com/airkey037/stqc_docs)) | MP3 48 kHz, also with 80 ms tones and 120 ms pauses |

The values of all sequences decoded from recordings match `pystqc.py`. Its
interpretation of the address/command pairs (format v2.16) yields `TEST` for
the test-call recordings and `ALARM` for the recording followed by a siren,
consistent with their content.

### Recordings

Sequences decoded from recordings of DSP-50 calls:

| Recording | Addresses (7 tones) | Commands (8 tones) | Other sequences |
|---|---|---|---|
| test calls, loudspeaker in a reverberant room, distorted | 10 | 10 | none |
| system test series, radio | 34 | 34 | 4-tone controller replies, one 10-tone sequence, 3-tone fragments |
| test calls, radio | 11 | 12 | first address cut by the start of the recording |
| alarm call between music and siren | 1 | 1 | none |

Resynthesizing the decoded sequences with `gen-ng` and decoding them again
gives identical results. Tone frequencies in the recordings deviate by at most
6 Hz from nominal.

The results are unchanged with added in-band noise and with −20 dB or +20 dB
gain, except for the reverberant recording clipped by +20 dB (16 of 20
sequences).

### Test suite

`test/run_tests.sh` contains:

- end-to-end tests `gen-ng -Q` → `multimon-ng -a STQC`: address, address and
  command, protocol example, JSON output, −60 dB level;
- sox-processed tests: 1000 Hz interfering tone at the level of the STQC
  tones, +30 dB overdrive, band-limited noise (300-3000 Hz, about +4 dB SNR;
  decoding works down to about −2 dB), a call following a 2 s tone at
  2105 Hz, an echo tail in the pause between address and command;
- tests that must not produce output: two-tone sequence, single STQC tone,
  white noise, 300-3000 Hz sweep;
- recording excerpts in `test/samples/STQC_dsp50_*.flac`: radio test call,
  radio test call with controller reply, alarm call between music and siren,
  test call recorded from a loudspeaker in a reverberant room.
