#ifndef TONES_H
#define TONES_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
  uint16_t *buffer;
  size_t length;
} AudioBuffer;

// init_tones allocates memory for and calculates a 12-note octave,
// based on an initial frequency f0 and the sample rate fs.
// It returns 1 on success and 0 on failure (probably not enough memory).
int init_tones(AudioBuffer tones[12], float f0, float fs);

#endif

