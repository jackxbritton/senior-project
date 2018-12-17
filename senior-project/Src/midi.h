#ifndef MIDI_H
#define MIDI_H

#include <stdint.h>

// MidiEventType is an enum for MIDI event types.
// These two are the only ones we support.
// They are also not one-to-one with the MIDI spec.
typedef enum {
  MIDI_EVENT_NOTE, // Represents both note on and note off.
  MIDI_EVENT_VOLUME
} MidiEventType;

// MidiEvent is a tagged union for MIDI event data.
// This is the poor man's polymorphism.
typedef struct {

  MidiEventType type;

  union {

      // MIDI_EVENT_NOTE.
      struct {
        uint8_t key;
        uint8_t velocity;
      } note;

      // MIDI_EVENT_VOLUME.
      uint8_t volume;

  } data;

} MidiEvent;

// To initialize a MidiInterpreter instance,
// just make sure it's zeroed.
// Its contents should *not* be accessed outside of midi_step.
typedef struct {
  uint8_t buf[3];
  int buf_count;
  int event_size;
  MidiEvent event;
} MidiInterpreter;

// midi_step advances an interpreter with a byte of input.
// It returns a pointer to an event when one is ready,
// and NULL otherwise.
MidiEvent *midi_step(MidiInterpreter *mi, uint8_t byte);

#endif

