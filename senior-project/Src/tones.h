#ifndef TONES_H
#define TONES_H

#include <stdint.h>

typedef struct {
  uint16_t *buf;
  int buf_len;
} Tone;

extern const Tone tones[12];

typedef struct {
  float *buf;
  int buf_len;
} AudioBuffer;

int generate_octave(
  AudioBuffer bufs[12],
  float timer_clock_frequency,
  int arr,
  float f0
);

#endif

