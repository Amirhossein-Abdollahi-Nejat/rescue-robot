import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import math
import RPi.GPIO as GPIO
from time import sleep

# Define motor GPIO pins
LEFT_MOTOR1 = 17
LEFT_MOTOR2 = 18
RIGHT_MOTOR1 = 22
RIGHT_MOTOR2 = 23

# MPU6050 setup (using smbus)
import smbus

# MPU6050 Register addresses
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_X = 0x3B
ACCEL_Y = 0x3D
ACCEL_Z = 0x3F
GYRO_X = 0x43
GYRO_Y = 0x45
GYRO_Z = 0x47

# Initialize I2C bus
bus = smbus.SMBus(1)
bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup([LEFT_MOTOR1, LEFT_MOTOR2, RIGHT_MOTOR1, RIGHT_MOTOR2], GPIO.OUT)

# Function to read raw accelerometer data
def read_raw_data(addr):
    high = bus.read_byte_data(MPU6050_ADDR, addr)
    low = bus.read_byte_data(MPU6050_ADDR, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value -= 65536
    return value

# Function to calculate orientation (roll, pitch, yaw)
def get_orientation():
    accel_x = read_raw_data(ACCEL_X)
    accel_y = read_raw_data(ACCEL_Y)
    accel_z = read_raw_data(ACCEL_Z)
    gyro_x = read_raw_data(GYRO_X)
    gyro_y = read_raw_data(GYRO_Y)
    gyro_z = read_raw_data(GYRO_Z)
    
    # Calculate roll and pitch
    roll = math.atan2(accel_y, accel_z) * 180.0 / math.pi
    pitch = math.atan(-accel_x / math.sqrt(accel_y * accel_y + accel_z * accel_z)) * 180.0 / math.pi
    
    return roll, pitch, gyro_x, gyro_y, gyro_z

# Define the ROS2 Node
class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self.cmd_subscriber = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.roll, self.pitch, self.gyro_x, self.gyro_y, self.gyro_z = 0, 0, 0, 0, 0
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.get_logger().info("Robot Controller Node Started!")

    def cmd_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def control_loop(self):
        self.roll, self.pitch, self.gyro_x, self.gyro_y, self.gyro_z = get_orientation()

        # Handle tilt scenarios
        if abs(self.roll) > 15 or abs(self.pitch) > 15:
            self.get_logger().info(f"Orientation issue detected! Roll: {self.roll}, Pitch: {self.pitch}")
            self.stop_movement()
            self.get_logger().info("Robot stopped due to excessive tilt!")

        # Stabilize robot when tilt is small
        elif abs(self.roll) > 5 or abs(self.pitch) > 5:
            self.stabilize_robot()

        # Correct robot orientation if it drifts
        elif abs(self.roll) > 10 or abs(self.pitch) > 10:
            self.correct_orientation()

        # Control the motors based on twist commands
        if self.linear_x > 0.5:
            self.move_forward()
        elif self.linear_x < -0.5:
            self.move_backward()
        if self.angular_z > 0:
            self.turn_left()
        elif self.angular_z < 0:
            self.turn_right()

    def stop_movement(self):
        GPIO.output(LEFT_MOTOR1, GPIO.LOW)
        GPIO.output(LEFT_MOTOR2, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR2, GPIO.LOW)

    def move_forward(self):
        GPIO.output(LEFT_MOTOR1, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR2, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR2, GPIO.LOW)

    def move_backward(self):
        GPIO.output(LEFT_MOTOR1, GPIO.LOW)
        GPIO.output(LEFT_MOTOR2, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR2, GPIO.HIGH)

    def turn_left(self):
        GPIO.output(LEFT_MOTOR1, GPIO.LOW)
        GPIO.output(LEFT_MOTOR2, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR1, GPIO.HIGH)
        GPIO.output(RIGHT_MOTOR2, GPIO.LOW)

    def turn_right(self):
        GPIO.output(LEFT_MOTOR1, GPIO.HIGH)
        GPIO.output(LEFT_MOTOR2, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR1, GPIO.LOW)
        GPIO.output(RIGHT_MOTOR2, GPIO.HIGH)

    def stabilize_robot(self):
        # Example stabilization logic based on pitch/roll
        self.get_logger().info(f"Stabilizing robot, Pitch: {self.pitch}, Roll: {self.roll}")
        self.move_forward()  # Move robot in a balanced manner

    def correct_orientation(self):
        if self.roll > 10:
            self.turn_right()
            self.get_logger().info("Correcting roll orientation!")
        elif self.pitch > 10:
            self.turn_left()
            self.get_logger().info("Correcting pitch orientation!")

def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
