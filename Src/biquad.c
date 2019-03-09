#include "biquad.h"
#include <math.h>

float biquad_filter_process(BiquadFilter *filter, float in) {
	float out = in*filter->b0 + filter->s1;
	filter->s1 = in*filter->b1 - out*filter->a1 + filter->s2;
	filter->s2 = in*filter->b2 - out*filter->a2;
	return out;
}

void biquad_filter_low_pass(BiquadFilter *filter, int sample_rate, float fc, float q) {
	float omega = 2.0f*M_PI*fc/sample_rate;
	float c = cosf(omega), s = sinf(omega);
	float alpha = s / (2.0f*q);
	float a0 = 1.0f + alpha;
	filter->b0 = 1.0f/a0 * (1.0f - c)/2.0f;
	filter->b1 = 1.0f/a0 * (1.0f - c);
	filter->b2 = 1.0f/a0 * (1.0f - c)/2.0f;
	filter->a1 = 1.0f/a0 * -2.0f*c;
	filter->a2 = 1.0f/a0 * (1.0f - alpha);
	filter->s1 = 0.0f;
	filter->s2 = 0.0f;
}
