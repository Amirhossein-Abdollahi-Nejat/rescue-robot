import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt

# Low-pass filter design
def butter_lowpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b, a

# High-pass filter design
def butter_highpass(cutoff, fs, order=5):
    nyquist = 0.5 * fs
    normal_cutoff = cutoff / nyquist
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    return b, a

# Apply low-pass filter
def butter_lowpass_filter(data, cutoff, fs, order=5):
    b, a = butter_lowpass(cutoff, fs, order)
    y = filtfilt(b, a, data)
    return y

# Apply high-pass filter
def butter_highpass_filter(data, cutoff, fs, order=5):
    b, a = butter_highpass(cutoff, fs, order)
    y = filtfilt(b, a, data)
    return y

# Example parameters
fs = 100.0  # Sampling frequency (Hz)
low_cutoff = 1.0  # Cutoff frequency for low-pass filter (Hz)
high_cutoff = 5.0  # Cutoff frequency for high-pass filter (Hz)
t = np.linspace(0, 1.0, int(fs))  # Time vector (1 second of data)

# Generate example signal (combination of low and high frequency signals)
# Low-frequency signal (simulating Earth shaking or slow movement)
low_freq = np.sin(2 * np.pi * 0.1 * t)
# High-frequency noise (simulating robot movement or other high-frequency noise)
high_freq = 0.5 * np.random.randn(len(t))

# Combine both signals to create a noisy signal
signal = low_freq + high_freq

# Apply low-pass filter to remove high-frequency noise
filtered_lowpass_signal = butter_lowpass_filter(signal, low_cutoff, fs)

# Apply high-pass filter to remove low-frequency signal (keeping high-frequency components)
filtered_highpass_signal = butter_highpass_filter(signal, high_cutoff, fs)

# Plot the original, low-pass filtered, and high-pass filtered signals
plt.figure(figsize=(10, 8))

# Original noisy signal
plt.subplot(3, 1, 1)
plt.plot(t, signal, label='Original Noisy Signal')
plt.title('Original Noisy Signal')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.legend()

# Low-pass filtered signal (remove high-frequency noise)
plt.subplot(3, 1, 2)
plt.plot(t, filtered_lowpass_signal, label='Low-pass Filtered Signal', color='orange')
plt.title('Low-pass Filtered Signal (Remove High-Frequency Noise)')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.legend()

# High-pass filtered signal (remove low-frequency signal)
plt.subplot(3, 1, 3)
plt.plot(t, filtered_highpass_signal, label='High-pass Filtered Signal', color='green')
plt.title('High-pass Filtered Signal (Remove Low-Frequency Signal)')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.legend()

plt.tight_layout()
plt.show()
