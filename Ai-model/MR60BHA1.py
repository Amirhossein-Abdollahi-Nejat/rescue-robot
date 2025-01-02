import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tensorflow as tf
import numpy as np
import serial
import time

class HumanAnimalDetectionNode(Node):
    def __init__(self):
        super().__init__('human_animal_detection_node')
        self.publisher_ = self.create_publisher(String, 'detection_results', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)  # Publish every 0.5 seconds

        self.get_logger().info("Starting MR60BHA1 Sensor Node...")

        # Initialize serial connection to the sensor
        try:
            self.sensor = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
            time.sleep(2)  # Wait for the connection to establish
            self.get_logger().info("Sensor connected successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to the sensor: {e}")
            raise

        # Load the AI model
        try:
            self.model = tf.keras.models.load_model('human_detection_model.h5')
            self.get_logger().info("AI model loaded successfully. Ready for classification.")
        except Exception as e:
            self.get_logger().error(f"Failed to load AI model: {e}")
            raise

    def timer_callback(self):
        try:
            # Read sensor data (example: "1.2,0.85,72.0,18.0")
            self.sensor.write(b'R')  # Example command to trigger a read
            raw_data = self.sensor.readline().decode('utf-8').strip()

            if not raw_data:
                self.get_logger().error("Sensor data timeout. Retrying...")
                return

            # Parse sensor data
            distance, motion_intensity, heart_rate, breathing_rate = map(float, raw_data.split(','))
            self.get_logger().info(f"Sensor Data: Distance={distance}m, Motion Intensity={motion_intensity}, Heart Rate={heart_rate} bpm, Breathing Rate={breathing_rate} bpm")

            # Prepare data for prediction
            input_data = np.array([[distance, motion_intensity, heart_rate, breathing_rate]])
            prediction = self.model.predict(input_data)
            confidence = prediction[0][0]

            # Interpret prediction
            if confidence > 0.5:
                label = "HUMAN"
                confidence_percentage = confidence * 100
            else:
                label = "ANIMAL"
                confidence_percentage = (1 - confidence) * 100

            self.get_logger().info(f"AI Prediction: {label} DETECTED (Confidence: {confidence_percentage:.2f}%)")

            # Publish prediction
            result_msg = String()
            result_msg.data = (
                f"{{\"distance\": {distance}, \"motion_intensity\": {motion_intensity}, "
                f"\"heart_rate\": {heart_rate}, \"breathing_rate\": {breathing_rate}, "
                f"\"prediction\": \"{label}\", \"confidence\": {confidence_percentage:.2f}}}" 
            )
            self.publisher_.publish(result_msg)
        except Exception as e:
            self.get_logger().error(f"Error processing sensor data: {e}")

    def destroy_node(self):
        super().destroy_node()
        if self.sensor:
            self.sensor.close()
            self.get_logger().info("Sensor disconnected.")


# Entry point for the ROS2 node
def main(args=None):
    rclpy.init(args=args)
    node = HumanAnimalDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down gracefully...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
