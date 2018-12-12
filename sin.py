#!/usr/bin/env python

import sys
import numpy as np
import matplotlib.pyplot as plt

n = 256
t = np.arange(0, n) / n
x = (np.sin(2*np.pi*t) + 1)/2 * 4096

plt.plot(t, x)
plt.show()
for i in x:
    print('  %d,' % i)
print(len(x))
sys.exit()






# Define a few constants.
resolution = 4096
timer_frequency = 84e6
arr = 2047
sample_rate = timer_frequency / (arr + 1)
sample_period = 1 / sample_rate

# Start at a1 (55Hz) and go up to g#2.
a1 = 55
frequencies = a1 * 2**(np.arange(0, 12)/12)

for i, f in enumerate(frequencies):

    # How many samples do we need?
    n = sample_rate / f
    t = np.arange(0, n) / n
    x = (np.sin(2*np.pi*t) + 1)/2 * resolution
    x = x.astype(int)

    # Print out as C code.
    print('const uint16_t tone%d[%d] = {' % (i, len(t)))
    for j, v in enumerate(x):
        sys.stdout.write(' %d' % v)
        if j != len(x)-1:
            sys.stdout.write(', ')
        if j % 8 == 0 and j != 0:
            print('')
    print('')
    print('};')
    print('')

