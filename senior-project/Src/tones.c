#include "tones.h"
#include <stdlib.h>
#include <math.h>
#define M_PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062f

int init_tones(AudioBuffer tones[12], float f0, float fs) {

  for (int i = 0; i < 12; i++) {
    
    // Calculate the buffer size for a complete sine wave at the given frequency.
    float f = f0 * powf(2.0f, (float) i/12);
    tones[i].length = roundf(fs / f);

    // Allocate memory.
    // If it fails, clean up and return 0.
    tones[i].buffer = malloc(tones[i].length * sizeof(uint16_t));
    if (tones[i].buffer == NULL) {
      for (int j = 0; j < i; j++) free(tones[j].buffer);
      return 0;
    }

    // Fill the buffer.
    for (int j = 0; j < tones[i].length; j++) {
      float angle = 2*M_PI * (float) j/tones[i].length;
      tones[i].buffer[j] = roundf(4095 * (sinf(angle) + 1.0f)/2.0f);
    }
  }

  return 1;

}

