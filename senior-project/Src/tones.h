#ifndef TONES_H
#define TONES_H

#include <stdint.h>

typedef struct {
  uint16_t *buf;
  int buf_len;
} Tone;

extern const Tone tones[12];


#endif

