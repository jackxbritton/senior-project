#ifndef TONES_H
#define TONES_H

#include <stdint.h>

typedef struct {
  const uint16_t *buf;
  const int buf_len;
} Tone;

extern const Tone tones[12];


#endif

