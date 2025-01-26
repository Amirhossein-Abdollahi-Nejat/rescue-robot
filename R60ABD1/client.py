import socket
import matplotlib.pyplot as plt
import numpy as np
import time

# Connect to the server
def connect_to_server(host="192.168.1.107", port=5000):
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))
    print(f"Connected to server at {host}:{port}")
    return client_socket

# Initialize the plot
def init_plot():
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    fig.tight_layout(pad=4.0)

    # Heart rate plot
    heart_rate_ax = axs[0, 0]
    heart_rate_ax.set_title("Heart Rate (BPM)")
    heart_rate_ax.set_ylim([0, 200])
    heart_rate_ax.set_xlabel("Time (s)")
    heart_rate_ax.set_ylabel("BPM")
    heart_rate_line, = heart_rate_ax.plot([], [], label="Heart Rate", color='r')

    # Breathing rate plot
    breath_rate_ax = axs[0, 1]
    breath_rate_ax.set_title("Breathing Rate (BPM)")
    breath_rate_ax.set_ylim([0, 40])  # Normal range for breathing rate
    breath_rate_ax.set_xlabel("Time (s)")
    breath_rate_ax.set_ylabel("BPM")
    breath_rate_line, = breath_rate_ax.plot([], [], label="Breathing Rate", color='g')

    # Motion level plot
    motion_ax = axs[1, 0]
    motion_ax.set_title("Motion Level")
    motion_ax.set_ylim([0, 100])
    motion_ax.set_xlabel("Time (s)")
    motion_ax.set_ylabel("Level")
    motion_line, = motion_ax.plot([], [], label="Motion Level", color='b')

    # Heart rate text display at bottom-right
    heart_rate_display_ax = axs[1, 1]
    heart_rate_display_ax.axis("off")  # Hide axes
    heart_rate_text = heart_rate_display_ax.text(
        0.5, 0.5, "Heart Rate: -- BPM", fontsize=20, ha='center', va='center', color='red', fontweight='bold'
    )

    return fig, heart_rate_line, breath_rate_line, motion_line, axs, heart_rate_text

# Update the plots
def update_plots(time_data, heart_rate, breath_rate, motion_level, heart_rate_line, breath_rate_line, motion_line, axs, heart_rate_text):
    # Set dynamic x-limits (scrolling effect)
    if len(time_data) > 50:
        axs[0, 0].set_xlim(time_data[-50], time_data[-1])
        axs[0, 1].set_xlim(time_data[-50], time_data[-1])
        axs[1, 0].set_xlim(time_data[-50], time_data[-1])

    # Update heart rate plot
    heart_rate_line.set_data(time_data, heart_rate)

    # Update breathing rate plot
    breath_rate_line.set_data(time_data, breath_rate)

    # Update motion level plot
    motion_line.set_data(time_data, motion_level)

    # Update heart rate text display
    if heart_rate:
        heart_rate_text.set_text(f"Heart Rate: {heart_rate[-1]} BPM")

    plt.draw()
    plt.pause(0.01)  # Faster updates

# Main function
def main():
    client_socket = connect_to_server()
    
    # Initialize the plots
    fig, heart_rate_line, breath_rate_line, motion_line, axs, heart_rate_text = init_plot()

    heart_rate_data = []
    breath_rate_data = []
    motion_data = []
    time_data = []

    start_time = time.time()

    while True:
        # Non-blocking socket read
        client_socket.settimeout(0.1)  # Prevent blocking
        try:
            data = client_socket.recv(1024).decode('utf-8')
            if data:
                print("Received data:", data)
                
                # Parse the received data
                lines = data.split('\n')
                
                heart_rate = 0
                breath_rate = 0
                motion_level = 0

                for line in lines:
                    if "Heart Rate" in line:
                        heart_rate = int(line.split(":")[1].strip().split()[0])
                    elif "Breathing Rate" in line:
                        breath_rate = int(line.split(":")[1].strip().split()[0])
                    elif "Motion Level" in line:
                        motion_level = int(line.split(":")[1].strip().split()[0])

                # Update the data lists
                current_time = time.time() - start_time
                time_data.append(current_time)
                heart_rate_data.append(heart_rate)
                breath_rate_data.append(breath_rate)
                motion_data.append(motion_level)

                # Keep only the latest 100 data points
                if len(time_data) > 100:
                    time_data.pop(0)
                    heart_rate_data.pop(0)
                    breath_rate_data.pop(0)
                    motion_data.pop(0)

                # Update the plots
                update_plots(time_data, heart_rate_data, breath_rate_data, motion_data, heart_rate_line, breath_rate_line, motion_line, axs, heart_rate_text)

        except socket.timeout:
            pass

        # Reduce CPU usage
        time.sleep(0.05)

    client_socket.close()

if __name__ == "__main__":
    main()
