#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt

# Define a few constants.
resolution = 4096
timer_frequency = 84e6
arr = 2047
sample_rate = timer_frequency / (arr + 1)

# Start at a1 (55Hz) and go up to g#2.
a1 = 55
frequencies = a1 * 2**(np.arange(0, 12)/12)

# Write the .c file.

sys.stdout.write('#include "tones.h"\n')
for i, f in enumerate(frequencies):

    # How many samples do we need?
    n = sample_rate / f
    t = np.arange(1, n) / n
    x = (np.sin(2*np.pi*t) + 1)/2 * 4095
    x = x.astype(int)

    # Print out the array (and format it nicely).
    sys.stdout.write('\nconst uint16_t tone%d[%d] = {' % (i, len(t)))
    for j, v in enumerate(x):
        if j % 6 == 0:
            sys.stdout.write('\n ')
        sys.stdout.write(' %d' % v)
        if j != len(x)-1:
            sys.stdout.write(',')
    sys.stdout.write('\n};\n')

