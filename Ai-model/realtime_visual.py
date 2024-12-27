import socket
import matplotlib.pyplot as plt
from collections import deque

# Socket setup (Laptop as the client)
HOST = '192.168.1.10'  # Replace with your Raspberry Pi's IP address
PORT = 12345  # Port used by the Raspberry Pi server

# Real-time plotting setup
max_data_points = 100  # Number of points to display in the graph at any time
time_window = deque(maxlen=max_data_points)
distance_data = deque(maxlen=max_data_points)
motion_data = deque(maxlen=max_data_points)
heart_rate_data = deque(maxlen=max_data_points)
breathing_rate_data = deque(maxlen=max_data_points)

# Function to receive data from the Raspberry Pi and update plots
def receive_and_plot_data():
    # Create a socket and connect to the Raspberry Pi server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))

        plt.ion()  # Turn on interactive mode for real-time plotting
        fig, axs = plt.subplots(2, 2, figsize=(12, 8))  # 2x2 grid of subplots
        axs = axs.ravel()  # Flatten the axes array for easy indexing

        while True:
            try:
                # Receive data from the Raspberry Pi
                data = s.recv(1024).decode('utf-8')  # Read the incoming data
                if data:
                    # Parse the incoming data
                    data_values = data.split(',')
                    timestamp = float(data_values[0])
                    distance = float(data_values[1])
                    motion = float(data_values[2])
                    heart_rate = float(data_values[3])
                    breathing_rate = float(data_values[4])
                    label = int(data_values[5])

                    # Update the data buffers
                    time_window.append(timestamp)
                    distance_data.append(distance)
                    motion_data.append(motion)
                    heart_rate_data.append(heart_rate)
                    breathing_rate_data.append(breathing_rate)

                    # Clear the previous plots and update with new data
                    axs[0].cla()
                    axs[0].plot(time_window, distance_data, label="Distance")
                    axs[0].set_title("Distance")
                    axs[0].set_xlabel("Time (s)")
                    axs[0].set_ylabel("Distance (m)")

                    axs[1].cla()
                    axs[1].plot(time_window, motion_data, label="Motion Intensity", color='orange')
                    axs[1].set_title("Motion Intensity")
                    axs[1].set_xlabel("Time (s)")
                    axs[1].set_ylabel("Motion Intensity")

                    axs[2].cla()
                    axs[2].plot(time_window, heart_rate_data, label="Heart Rate", color='green')
                    axs[2].set_title("Heart Rate")
                    axs[2].set_xlabel("Time (s)")
                    axs[2].set_ylabel("Heart Rate (bpm)")

                    axs[3].cla()
                    axs[3].plot(time_window, breathing_rate_data, label="Breathing Rate", color='red')
                    axs[3].set_title("Breathing Rate")
                    axs[3].set_xlabel("Time (s)")
                    axs[3].set_ylabel("Breathing Rate (bpm)")

                    # Adjust layout and redraw the plots
                    plt.tight_layout()
                    plt.pause(0.1)  # Pause for a brief moment to update the plots

            except KeyboardInterrupt:
                print("Data collection stopped.")
                break
            except Exception as e:
                print(f"Error: {e}")
                continue

if __name__ == "__main__":
    print("Starting data visualization...")
    receive_and_plot_data()
