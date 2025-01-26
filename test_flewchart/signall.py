import numpy as np
import matplotlib.pyplot as plt

# تبدیل داده‌های hexadecimal به باینری
def hex_to_bin(hex_str):
    return np.array(list(bytes.fromhex(hex_str)), dtype=np.uint8)

# تابع FFT برای تجزیه و تحلیل سیگنال
def analyze_signal(data):
    # اعمال FFT
    fft_result = np.fft.fft(data)
    # محاسبه دامنه (Magnitude)
    magnitude = np.abs(fft_result)
    # نمایش نتایج
    plt.plot(magnitude)
    plt.title("FFT Analysis of Signal")
    plt.xlabel("Frequency")
    plt.ylabel("Magnitude")
    plt.show()

# داده‌های خام (نمونه)
raw_data_hex = "535980030001013154435359800400020047795443"

# تبدیل به داده‌های باینری
raw_data = hex_to_bin(raw_data_hex)

# تحلیل سیگنال
analyze_signal(raw_data)
