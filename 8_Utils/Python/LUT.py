import numpy as np

# Define parameters
fs = 120e6  # Sampling frequency
Ts = 1 / fs  # Sampling time
Tb = 1e-6  # Burst time
Tau = 30e-6  # Pulse repetition time
fmax = 15e6  # Maximum frequency on ramp
fmin = 1e6  # Minimum frequency on ramp

# Compute number of samples per ramp
n = int(Tb / Ts)
N = np.arange(0, n, 1)

# Compute instantaneous phase
theta_n = 2 * np.pi * ((N**2 * Ts**2 * (fmax - fmin) / (2 * Tb)) + fmin * N * Ts)

# Generate waveform and scale it to 8-bit unsigned values (0 to 255)
y = 1 + np.sin(theta_n)  # Normalize from 0 to 2
y_scaled = np.round(y * 127.5).astype(int)  # Scale to 8-bit range (0-255)

# Print values in Verilog-friendly format
for _i in range(n):
    print(f"8'd{y_scaled[_i]},")
