#ifndef MIDI_H
#define MIDI_H

#include <stdint.h>

// MidiInterpreter doesn't actually do much,
// you just feed it bytes until it tells you it's
// read a full MIDI event.

// Zero to initialize!

typedef struct {

  // Read buf up to count to interpret the event.
  uint8_t buf[3];
  int count;

  // Don't touch this!
  int spec_size;

} MidiInterpreter;

// midi_step advances the interpreter with a byte of input.
// It returns 1 when the event has been completed,
// and 0 otherwise.
// If 1 is returned, read mi->buf up to mi->count for the data.
int midi_step(MidiInterpreter *mi, uint8_t byte);

#endif

