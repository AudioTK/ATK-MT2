import numpy as np
import sys
import matplotlib.pyplot as plt

data = np.loadtxt(sys.argv[1])

NFFT = 2048
Fs = 48000
noverlap = 1024
vmin = -50
vmax = 0

fig = plt.figure(num=None, figsize=(8, 6), dpi=160, facecolor='w', edgecolor='k')

plt.subplot(2, 1, 1)
pxx,  freq, t, cax = plt.specgram(data[:,0], NFFT=NFFT, Fs=Fs, noverlap=noverlap, vmin=vmin, vmax=vmax)
plt.xlabel("s")
plt.ylabel("Hz")
fig.colorbar(cax)

plt.subplot(2, 1, 2)
pxx,  freq, t, cax = plt.specgram(data[:,1], NFFT=NFFT, Fs=Fs, noverlap=noverlap, vmin=vmin, vmax=vmax)
plt.xlabel("s")
plt.ylabel("Hz")
fig.colorbar(cax)

plt.savefig(sys.argv[2])
