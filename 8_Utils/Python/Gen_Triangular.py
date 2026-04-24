import numpy as np
from numpy.fft import fft
import matplotlib.pyplot as plt


fs=125*pow(10,6) #sampling frequency
Ts=1/fs # sampling time
Tb=0.25*pow(10,-6) # burst time
Tau=1.5*pow(10,-6) # pulse repetition time
fmax=32*pow(10,6) # maximum frequency on ramp
fmin=1*pow(10,6) # minimum frequency on ramp
n=int(Tb/Ts) # number of samples per ramp
N = np.arange(0, n, 1)
theta_n= 2*np.pi*(pow(N,2)*pow(Ts,2)*(fmax-fmin)/(2*Tb)+fmin*N*Ts) # instantaneous phase
y = 1 + np.sin(theta_n) # ramp signal in time domain

M = np.arange(n, 2*n, 1)
theta_m= (
    2*np.pi*(pow(M,2)*pow(Ts,2)*(-fmax+fmin)/(2*Tb)+(-fmin+2*fmax)*M*Ts)
    - 2*np.pi*((fmin-fmax)*Tb/2+(2*fmax-fmin)*Tb)
) # instantaneous phase
z = 1 + np.sin(theta_m) # ramp signal in time domain

x = np.concatenate((y, z))

t = Ts*np.arange(0,2*n,1)
X = fft(x)
L =len(X)
freq_indices = np.arange(L)
T = L*Ts
freq = freq_indices/T


plt.figure(figsize = (12, 6))
plt.subplot(121)
plt.stem(freq, np.abs(X), 'b', \
         markerfmt=" ", basefmt="-b")
plt.xlabel('Freq (Hz)')
plt.ylabel('FFT Amplitude |Y(freq)|')
plt.xlim(0, (fmax+fmax/10))
plt.ylim(0, 20)

plt.subplot(122)
plt.plot(t, x)
plt.xlabel('Time (s)')
plt.ylabel('Amplitude')
plt.tight_layout()

plt.show()
