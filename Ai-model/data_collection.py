import serial
import socket
import csv
import time
import matplotlib.pyplot as plt
from collections import deque

# Configure the serial port (adjust based on your setup)
SERIAL_PORT = "/dev/ttyUSB0"  # Replace with your actual serial port
BAUD_RATE = 115200

# Socket setup (Raspberry Pi as the server)
host = '0.0.0.0'  # Accept connections from any IP address
port = 12345  # Port for the connection

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind((host, port))
server_socket.listen(1)

print("Waiting for a connection...")
connection, address = server_socket.accept()
print(f"Connection established with {address}")

# CSV file to save data
FILE_NAME = "sensor_data.csv"
FIELDS = ["Timestamp", "Distance", "Motion Intensity", "Heart Rate", "Breathing Rate", "Label"]

# Real-time plotting setup
max_data_points = 100  # Number of points to display in the graph at any time
time_window = deque(maxlen=max_data_points)
distance_data = deque(maxlen=max_data_points)
motion_data = deque(maxlen=max_data_points)
heart_rate_data = deque(maxlen=max_data_points)
breathing_rate_data = deque(maxlen=max_data_points)

# Function to collect data from the sensor and send it to the laptop for a specified duration (in minutes)
def collect_and_send_data(label, duration_minutes):
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Collecting data with label: {label} for {duration_minutes} minutes...")
        
        # Open the CSV file in append mode
        with open(FILE_NAME, mode="a", newline="") as file:
            writer = csv.writer(file)
            
            # Write header if the file is empty
            if file.tell() == 0:
                writer.writerow(FIELDS)

            start_time = time.time()
            end_time = start_time + (duration_minutes * 60)  # Convert minutes to seconds

            plt.ion()  # Turn on interactive mode for real-time plotting
            fig, axs = plt.subplots(2, 2, figsize=(12, 8))  # 2x2 grid of subplots
            axs = axs.ravel()  # Flatten the axes array for easy indexing

            while time.time() < end_time:
                try:
                    # Read and parse the sensor data from the serial port
                    data_line = ser.readline().decode("utf-8").strip()
                    values = data_line.split(",")  # Adjust parsing based on your sensor's data format
                    if len(values) >= 4:
                        timestamp = time.time()
                        distance = float(values[0])
                        motion = float(values[1])
                        heart_rate = float(values[2])
                        breathing_rate = float(values[3])

                        # Write the data to the CSV file
                        writer.writerow([timestamp, distance, motion, heart_rate, breathing_rate, label])
                        print(f"Logged data: {distance}, {motion}, {heart_rate}, {breathing_rate}, Label: {label}")

                        # Send the data to the laptop (socket communication)
                        sensor_data = f"{timestamp},{distance},{motion},{heart_rate},{breathing_rate},{label}\n"
                        connection.send(sensor_data.encode())

                        # Update the data buffers
                        time_window.append(timestamp)
                        distance_data.append(distance)
                        motion_data.append(motion)
                        heart_rate_data.append(heart_rate)
                        breathing_rate_data.append(breathing_rate)

                        # Update the plots
                        axs[0].cla()  # Clear the previous plot
                        axs[0].plot(time_window, distance_data, label="Distance")
                        axs[0].set_title("Distance")
                        axs[0].set_xlabel("Time")
                        axs[0].set_ylabel("Distance (m)")

                        axs[1].cla()  # Clear the previous plot
                        axs[1].plot(time_window, motion_data, label="Motion Intensity", color='orange')
                        axs[1].set_title("Motion Intensity")
                        axs[1].set_xlabel("Time")
                        axs[1].set_ylabel("Motion")

                        axs[2].cla()  # Clear the previous plot
                        axs[2].plot(time_window, heart_rate_data, label="Heart Rate", color='green')
                        axs[2].set_title("Heart Rate")
                        axs[2].set_xlabel("Time")
                        axs[2].set_ylabel("Heart Rate (bpm)")

                        axs[3].cla()  # Clear the previous plot
                        axs[3].plot(time_window, breathing_rate_data, label="Breathing Rate", color='red')
                        axs[3].set_title("Breathing Rate")
                        axs[3].set_xlabel("Time")
                        axs[3].set_ylabel("Breathing Rate (bpm)")

                        # Adjust layout and redraw the plots
                        plt.tight_layout()
                        plt.pause(0.1)  # Pause for a brief moment to update the plots

                except KeyboardInterrupt:
                    print("Data collection stopped.")
                    break
                except Exception as e:
                    print(f"Error during data collection or sending: {e}")
                    continue

            print(f"Data collection for {duration_minutes} minutes completed.")

    except Exception as e:
        print(f"Error setting up serial connection: {e}")

# Example usage
label = input("Enter label (1 for human, 0 for non-human): ")
duration_minutes = int(input("Enter the duration in minutes for data collection: "))
collect_and_send_data(label, duration_minutes)

# Close the connection after the process ends
connection.close()
server_socket.close()
