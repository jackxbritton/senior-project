#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt

#frequencies = a4 * 2**(np.arange(0, 12)/12)

# Define a few constants.
resolution = 4096
timer_frequency = 84e6
arr = 2047
sample_rate = timer_frequency / (arr + 1)
sample_period = 1 / sample_rate

# Try to generate an a4.
a4 = 440
f = a4

# How many samples do we need?
n = sample_rate / f
t = np.arange(0, n) / n
x = (np.sin(2*np.pi*t) + 1)/2 * resolution
x = x.astype(int)

#plt.plot(t*n, x)
#plt.show()

# TODO Print out as C code.

print('const uint16_t tone[%d] = {' % len(t))
for v in x:
    print('  {:d},'.format(v))
print('}')

