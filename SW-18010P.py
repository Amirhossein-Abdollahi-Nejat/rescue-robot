import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import RPi.GPIO as GPIO

class EarthShakingSensorNode(Node):
    def __init__(self):
        super().__init__('earth_shaking_sensor')
        self.publisher_ = self.create_publisher(Float32, 'earth_shaking_data', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Timer to read the sensor
        self.sensor_pin = 17  # GPIO pin for SW-18010P
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.sensor_pin, GPIO.IN)

    def timer_callback(self):
        vibration_signal = GPIO.input(self.sensor_pin)
        # Process the signal (e.g., applying the low-pass filter)
        filtered_signal = self.apply_filter(vibration_signal)
        msg = Float32()
        msg.data = filtered_signal
        self.publisher_.publish(msg)

    def apply_filter(self, signal):
        # Apply your filter here (using the low-pass/high-pass filter functions)
        # Assuming `signal` is an array of readings (this part will need adaptation)
        filtered_signal = butter_lowpass_filter(signal, cutoff=1.0, fs=100.0)
        return filtered_signal

def main(args=None):
    rclpy.init(args=args)
    node = EarthShakingSensorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
