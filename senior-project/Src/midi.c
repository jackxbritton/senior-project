#include "midi.h"
#include <string.h>

static int event_sizes[7] = { 3, 3, 3, 3, 2, 2, 3 };

int midi_step(MidiInterpreter *mi, uint8_t byte) {

  // Check for a leading 1 (a new event).
  if (byte >> 7) {

    // Push the byte.
    mi->buf[0] = byte;
    mi->count = 1;

    // Set the spec_size (size of the event according to the spec).
    int size_index = (byte >> 4) - 0x8;
    if (size_index >= 0 && size_index < 7) {
      mi->spec_size = event_sizes[(byte >> 4) - 0x8];
    }

    return 0;
  }

  // No events are over 3 bytes.
  if (mi->count == 3) return 0;

  // Push new byte.
  mi->buf[mi->count] = byte;
  mi->count++;

  // Maybe we finished the event.
  if (mi->count == mi->spec_size) {
    return 1;
  }

  return 0;

}

