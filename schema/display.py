import numpy as np
import sys
import matplotlib.pyplot as plt

cmap = plt.cm.cubehelix

data = np.loadtxt(sys.argv[1])

NFFT = 2048
Fs = 96000
noverlap = 512
vmin = -100
vmax = -20

fig = plt.figure(num=None, figsize=(8, 6), dpi=160, facecolor='w', edgecolor='k')

plt.subplot(2, 1, 1)
pxx1, freq1, t, cax = plt.specgram(data[:,0], NFFT=NFFT, Fs=Fs, noverlap=noverlap, vmin=vmin, vmax=vmax, cmap=cmap)
plt.xlabel("s")
plt.ylabel("Hz")
fig.colorbar(cax)

plt.subplot(2, 1, 2)
pxx2, freq2, t, cax = plt.specgram(data[:,1], NFFT=NFFT, Fs=Fs, noverlap=noverlap, vmin=vmin, vmax=vmax, cmap=cmap)
plt.xlabel("s")
plt.ylabel("Hz")
fig.colorbar(cax)

plt.savefig(sys.argv[2])

fig = plt.figure(num=None, figsize=(8, 6), dpi=160, facecolor='w', edgecolor='k')

bode = []

for i in range(pxx2.shape[1]):
  pos = np.argmax(pxx2[:, i])
  bode.append((freq2[pos], pxx2[pos, i] / pxx1[pos, i]))

data = np.array(bode).T
plt.semilogx(data[0, :], 10 * np.log(data[1, :]))
print(data)

plt.savefig("bode-" + sys.argv[2])
