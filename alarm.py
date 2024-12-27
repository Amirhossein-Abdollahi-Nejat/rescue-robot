import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32
from sensor_msgs.msg import Temperature, RelativeHumidity
import RPi.GPIO as GPIO
import time

class AlarmNode(Node):

    def __init__(self):
        super().__init__('alarm_node')

        # Initialize the GPIO for buzzer
        self.buzzer_pin = 18  # Connect your buzzer to GPIO 18
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.buzzer_pin, GPIO.OUT)

        # Subscribing to all sensor topics
        self.gas_subscriber = self.create_subscription(Float32, 'gas_concentration', self.gas_callback, 10)
        self.vibration_subscriber = self.create_subscription(Bool, 'vibration_detected', self.vibration_callback, 10)
        self.sound_subscriber = self.create_subscription(Int32, 'sound_level', self.sound_callback, 10)
        self.temp_subscriber = self.create_subscription(Temperature, 'temperature_data', self.temperature_callback, 10)
        self.humidity_subscriber = self.create_subscription(RelativeHumidity, 'humidity_data', self.humidity_callback, 10)

        # Assuming other sensors like the gas sensor or modules can also be added
        self.motion_detection_subscriber = self.create_subscription(Bool, 'motion_detected', self.motion_callback, 10)
        self.heart_rate_subscriber = self.create_subscription(Float32, 'heart_rate', self.heart_rate_callback, 10)
        
        # Initialize all sensor state variables
        self.gas_level = None
        self.vibration_detected = False
        self.sound_level = None
        self.temperature = None
        self.humidity = None
        self.motion_detected = False
        self.heart_rate = None

    def gas_callback(self, msg):
        self.gas_level = msg.data
        self.check_alarm_conditions()

    def vibration_callback(self, msg):
        self.vibration_detected = msg.data
        self.check_alarm_conditions()

    def sound_callback(self, msg):
        self.sound_level = msg.data
        self.check_alarm_conditions()

    def temperature_callback(self, msg):
        self.temperature = msg.temperature
        self.check_alarm_conditions()

    def humidity_callback(self, msg):
        self.humidity = msg.relative_humidity
        self.check_alarm_conditions()

    def motion_callback(self, msg):
        self.motion_detected = msg.data
        self.check_alarm_conditions()

    def heart_rate_callback(self, msg):
        self.heart_rate = msg.data
        self.check_alarm_conditions()

    def check_alarm_conditions(self):
        # Check if any sensor exceeds threshold, indicating danger
        if self.gas_level is not None and self.gas_level > 0.7:  # Adjust threshold as needed
            self.trigger_alarm("Gas leak detected!")
        elif self.vibration_detected:
            self.trigger_alarm("Vibration detected!")
        elif self.sound_level is not None and self.sound_level > 100:  # Threshold for loud sound
            self.trigger_alarm("Loud sound detected!")
        elif self.temperature is not None and self.temperature > 40:  # Example threshold for high temp
            self.trigger_alarm("High temperature detected!")
        elif self.humidity is not None and self.humidity < 20:  # Low humidity threshold
            self.trigger_alarm("Low humidity detected!")
        elif self.motion_detected:
            self.trigger_alarm("Human motion detected!")
        elif self.heart_rate is not None and self.heart_rate < 40:  # Too low heart rate
            self.trigger_alarm("Low heart rate detected!")

    def trigger_alarm(self, message):
        # Trigger the buzzer and log the alarm
        self.get_logger().info(f"ALARM! {message}")
        # Activate the buzzer
        GPIO.output(self.buzzer_pin, GPIO.HIGH)  # Turn buzzer on
        time.sleep(1)  # Keep buzzer on for 1 second (can adjust duration)
        GPIO.output(self.buzzer_pin, GPIO.LOW)  # Turn buzzer off

def main(args=None):
    rclpy.init(args=args)
    alarm_node = AlarmNode()
    rclpy.spin(alarm_node)
    alarm_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
