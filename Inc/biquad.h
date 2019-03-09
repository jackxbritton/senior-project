#ifndef BIQUAD_H
#define BIQUAD_H

typedef struct {
	float b0, b1, b2, a1, a2;
	float s1, s2;
} BiquadFilter;

float biquad_filter_process(BiquadFilter *filter, float in);

// https://music.columbia.edu/pipermail/music-dsp/2001-March/041752.html
void biquad_filter_low_pass(BiquadFilter *filter, int sample_rate, float fc, float q);

#endif

