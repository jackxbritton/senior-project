#include "midi.h"
#include <string.h>

static int event_sizes[7] = { 3, 3, 3, 3, 2, 2, 3 };

MidiEvent *midi_step(MidiInterpreter *mi, uint8_t byte) {

  // Check for a leading 1 (a new event).
  if (byte >> 7) {

    // Push the byte.
    mi->buf[0] = byte;
    mi->buf_count = 1;

    // Set the spec_size (size of the event according to the spec).
    int size_index = (byte >> 4) - 0x8;
    if (size_index >= 0 && size_index < 7) {
      mi->event_size = event_sizes[size_index];
    }

    return NULL;

  }

  // No events are over 3 bytes.
  if (mi->buf_count == 3) return NULL;

  // Push new byte.
  mi->buf[mi->buf_count] = byte;
  mi->buf_count++;

  // Maybe we finished the event.
  if (mi->buf_count == mi->event_size) {

    uint8_t type = mi->buf[0] >> 4;
    switch (type) {

    // Note off event.
    case 0x08:
      mi->event.type = MIDI_EVENT_NOTE;
      mi->event.data.note.key = mi->buf[1];
      mi->event.data.note.velocity = 0;
      break;

    // Note on event.
    case 0x09:
      mi->event.type = MIDI_EVENT_NOTE;
      mi->event.data.note.key = mi->buf[1];
      mi->event.data.note.velocity = mi->buf[2];
      break;

    // Control change.
    case 0x0b:
      if (mi->buf[1] == 0x07) {
        // Volume event.
        mi->event.type = MIDI_EVENT_VOLUME;
        mi->event.data.volume = mi->buf[2];
      }
      break;

    default:
      return NULL;

    }

    return &mi->event;

  }

  return NULL;

}

