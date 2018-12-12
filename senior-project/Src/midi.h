#ifndef MIDI_H
#define MIDI_H

#include <stdint.h>

#define DMA_BUF_LEN 1024
extern uint16_t dma_buf[DMA_BUF_LEN];
extern int midi_count;
extern int note;

extern int f_counter;

#endif

