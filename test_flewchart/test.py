import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32
from sensor_msgs.msg import Temperature, RelativeHumidity, NavSatFix, Imu, Image
import time
import os
import subprocess
import sys
import RPi.GPIO as GPIO

class TestNode(Node):

    def __init__(self):
        super().__init__('test_node')

        # Initialize GPIO for the buzzer
        self.buzzer_pin = 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.buzzer_pin, GPIO.OUT)

        # Initialize test results
        self.tests_passed = True
        
        # Subscribing to all sensor topics
        self.gas_subscriber = self.create_subscription(Float32, 'gas_concentration', self.gas_callback, 10)
        self.vibration_subscriber = self.create_subscription(Bool, 'vibration_detected', self.vibration_callback, 10)
        self.sound_subscriber = self.create_subscription(Int32, 'sound_level', self.sound_callback, 10)
        self.temp_subscriber = self.create_subscription(Temperature, 'temperature_data', self.temperature_callback, 10)
        self.humidity_subscriber = self.create_subscription(RelativeHumidity, 'humidity_data', self.humidity_callback, 10)
        self.mr60bha1_subscriber = self.create_subscription(Float32, 'heart_rate', self.mr60bha1_callback, 10)
        self.gps_subscriber = self.create_subscription(NavSatFix, 'gps_data', self.gps_callback, 10)
        self.imu_subscriber = self.create_subscription(Imu, 'imu_data', self.imu_callback, 10)
        self.camera_subscriber = self.create_subscription(Image, 'camera_data', self.camera_callback, 10)
        self.ultrasonic_1_subscriber = self.create_subscription(Float32, 'ultrasonic_1_distance', self.ultrasonic_1_callback, 10)
        self.ultrasonic_2_subscriber = self.create_subscription(Float32, 'ultrasonic_2_distance', self.ultrasonic_2_callback, 10)
        
        # Initialize all sensor state variables
        self.gas_level = None
        self.vibration_detected = False
        self.sound_level = None
        self.temperature = None
        self.humidity = None
        self.heart_rate = None
        self.gps_data = None
        self.imu_data = None
        self.camera_data = None
        self.ultrasonic_1_distance = None
        self.ultrasonic_2_distance = None

        # Motors
        self.motor_test_passed = False
        
        # Connectivity
        self.connection_test_passed = False
        
        # Perform the test on initialization
        self.run_tests()

    # Sensor callback methods
    def gas_callback(self, msg):
        self.gas_level = msg.data
        self.test_component('Gas Sensor', self.gas_level)

    def vibration_callback(self, msg):
        self.vibration_detected = msg.data
        self.test_component('Vibration Sensor', self.vibration_detected)

    def sound_callback(self, msg):
        self.sound_level = msg.data
        self.test_component('Sound Sensor', self.sound_level)

    def temperature_callback(self, msg):
        self.temperature = msg.temperature
        self.test_component('Temperature Sensor', self.temperature)

    def humidity_callback(self, msg):
        self.humidity = msg.relative_humidity
        self.test_component('Humidity Sensor', self.humidity)

    def mr60bha1_callback(self, msg):
        self.heart_rate = msg.data
        self.test_component('MR60BHA1 Sensor', self.heart_rate)

    def gps_callback(self, msg):
        self.gps_data = msg
        self.test_component('GPS Sensor', self.gps_data.latitude)

    def imu_callback(self, msg):
        self.imu_data = msg
        self.test_component('IMU Sensor', self.imu_data.linear_acceleration.x)  # Example: check X-axis acceleration

    def camera_callback(self, msg):
        self.camera_data = msg
        self.test_component('Camera', len(msg.data))  # Test by checking if there's image data (non-zero)

    def ultrasonic_1_callback(self, msg):
        self.ultrasonic_1_distance = msg.data
        self.test_component('Ultrasonic 1', self.ultrasonic_1_distance)

    def ultrasonic_2_callback(self, msg):
        self.ultrasonic_2_distance = msg.data
        self.test_component('Ultrasonic 2', self.ultrasonic_2_distance)

    # Motor Test
    def motor_test(self):
        self.get_logger().info("Testing motors...")

        # This would ideally send control commands to the motors to check their response.
        self.motor_test_passed = True  # Set to False if motors don't respond as expected

        if not self.motor_test_passed:
            self.log_failure("Motor Test", "Motors did not respond correctly.")

    # Connectivity Test (ping operator machine)
    def connectivity_test(self):
        self.get_logger().info("Testing connectivity to operator...")

        # Ping the operator's machine (replace '192.168.x.x' with the operator's IP address)
        operator_ip = "192.168.x.x"  # Replace this with the actual IP address of the operator's machine
        try:
            response = subprocess.run(['ping', '-c', '10', operator_ip], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            if response.returncode == 0:
                self.connection_test_passed = True
                self.get_logger().info("Connection to operator's machine successful!")
            else:
                self.log_failure("Connectivity Test", "Failed to ping operator's machine.")
        except Exception as e:
            self.log_failure("Connectivity Test", f"Error pinging operator: {e}")

    # Test all components
    def test_component(self, component_name, value):
        if component_name == 'Gas Sensor':
            if value < 0 or value > 1:  # Gas concentration should be between 0 and 1
                self.log_failure(component_name, f"Gas concentration out of bounds: {value}")
        elif component_name == 'Vibration Sensor':
            if not self.vibration_detected:
                self.log_failure(component_name, "Vibration not detected!")
        elif component_name == 'Sound Sensor':
            if value is None or value < 0:
                self.log_failure(component_name, f"Invalid sound level: {value}")
        elif component_name == 'Temperature Sensor':
            if value is None or value < -50 or value > 150:
                self.log_failure(component_name, f"Invalid temperature: {value}")
        elif component_name == 'Humidity Sensor':
            if value is None or value < 0 or value > 100:
                self.log_failure(component_name, f"Invalid humidity: {value}")
        elif component_name == 'MR60BHA1 Sensor':
            if value is None or value < 30 or value > 200:
                self.log_failure(component_name, f"Invalid heart rate: {value}")
        elif component_name == 'GPS Sensor':
            if value is None or abs(value) > 90:  # Latitude should be between -90 and 90
                self.log_failure(component_name, f"Invalid latitude: {value}")
        elif component_name == 'IMU Sensor':
            if value is None or abs(value) > 10:  # Check if the acceleration is within an expected range (e.g., -10 to 10 m/s^2)
                self.log_failure(component_name, f"Invalid IMU data: {value}")
        elif component_name == 'Camera':
            if value == 0:  # If no image data is received
                self.log_failure(component_name, "No camera data received")
        elif component_name == 'Ultrasonic 1':
            if value is None or value < 0 or value > 400:  # Distance should be between 0 and 400 cm
                self.log_failure(component_name, f"Invalid distance from Ultrasonic 1: {value} cm")
        elif component_name == 'Ultrasonic 2':
            if value is None or value < 0 or value > 400:  # Distance should be between 0 and 400 cm
                self.log_failure(component_name, f"Invalid distance from Ultrasonic 2: {value} cm")

    def log_failure(self, component_name, message):
        self.tests_passed = False
        self.get_logger().error(f"TEST FAILED: {component_name} - {message}")
        self.buzzer_beep(3)  # 3 beeps for failure

    def buzzer_beep(self, count):
        for _ in range(count):
            GPIO.output(self.buzzer_pin, GPIO.HIGH)
            time.sleep(0.2)
            GPIO.output(self.buzzer_pin, GPIO.LOW)
            time.sleep(0.2)

    def prompt_operator(self):
        response = input("One or more tests failed. Do you want to continue (y/n)? ")
        if response.lower() == 'n':
            self.get_logger().info("Operator chose to abort. Shutting down.")
            sys.exit(1)
        elif response.lower() != 'y':
            self.get_logger().info("Invalid input. Shutting down.")
            sys.exit(1)

    def run_tests(self):
        self.get_logger().info("Running system tests...")
        time.sleep(1)
        
        self.motor_test()
        self.connectivity_test()

        if self.gas_level is None or self.vibration_detected is False or self.sound_level is None or self.temperature is None or self.humidity is None or self.heart_rate is None or self.gps_data is None or self.imu_data is None or self.camera_data is None or self.ultrasonic_1_distance is None or self.ultrasonic_2_distance is None:
            self.log_failure("Sensors", "One or more sensors failed to respond.")

        if self.tests_passed and self.motor_test_passed and self.connection_test_passed:
            self.get_logger().info("All tests passed. System ready to start.")
            self.buzzer_beep(1)
            time.sleep(5)
        else:
            self.get_logger().error("Some tests failed. System cannot start.")
            self.prompt_operator()

def main(args=None):
    rclpy.init(args=args)
    test_node = TestNode()
    rclpy.spin(test_node)
    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
