import socket
import matplotlib.pyplot as plt
import numpy as np
import time

# Connect to the server
def connect_to_server(host="192.168.1.32", port=5005):
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
    heart_rate_ax.set_title("Heart Rate (BPM)", color='black')
    heart_rate_ax.set_ylim([0, 200])
    heart_rate_ax.set_xlabel("Time (s)", color='black')
    heart_rate_ax.set_ylabel("BPM", color='black')
    heart_rate_ax.set_facecolor("black")  # Set background color to black
    heart_rate_ax.grid(True, color='gray', linestyle='--', alpha=0.6)  # Light gray grid lines
    heart_rate_line, = heart_rate_ax.plot([], [], label="Heart Rate", color='r')

    # Breathing rate plot
    breath_rate_ax = axs[0, 1]
    breath_rate_ax.set_title("Breathing Rate (BPM)", color='black')
    breath_rate_ax.set_ylim([0, 40])  # Normal range for breathing rate
    breath_rate_ax.set_xlabel("Time (s)", color='black')
    breath_rate_ax.set_ylabel("BPM", color='black')
    breath_rate_ax.set_facecolor("black")  # Set background color to black
    breath_rate_ax.grid(True, color='gray', linestyle='--', alpha=0.6)  # Light gray grid lines
    breath_rate_line, = breath_rate_ax.plot([], [], label="Breathing Rate", color='b')

    # Motion level contour plot
    motion_ax = axs[1, 0]
    motion_ax.set_title("Motion Level Contour", color='black')
    motion_ax.set_xlabel("Time (s)", color='black')
    motion_ax.set_ylabel("Spatial Dimension", color='black')
    motion_levels = np.zeros((10, 50))  # Initialize a 10x50 grid
    motion_contour = motion_ax.contourf(motion_levels, levels=50, cmap="viridis")
    plt.colorbar(motion_contour, ax=motion_ax, label="Motion Level")

    # Heart rate, Breathing rate & Motion level text display at bottom-right
    heart_rate_display_ax = axs[1, 1]
    heart_rate_display_ax.axis("off")  # Hide axes
    
    heart_rate_text = heart_rate_display_ax.text(
        0.5, 0.7, "Heart Rate: -- BPM", fontsize=20, ha='center', va='center', 
        color='red', fontweight='bold'
    )

    breathing_rate_text = heart_rate_display_ax.text(
        0.5, 0.5, "Breathing Rate: -- BPM", fontsize=18, ha='center', va='center', 
        color='blue', fontweight='bold'
    )

    motion_level_text = heart_rate_display_ax.text(
        0.5, 0.3, "Motion Level: --", fontsize=18, ha='center', va='center', 
        color='green', fontweight='bold'
    )

    # Update tick label colors to match the theme
    for ax in [heart_rate_ax, breath_rate_ax, motion_ax]:
        ax.tick_params(axis='x', colors='black')
        ax.tick_params(axis='y', colors='black')

    return fig, heart_rate_line, breath_rate_line, motion_contour, motion_levels, axs, heart_rate_text, breathing_rate_text, motion_level_text

# Update the plots
def update_plots(time_data, heart_rate, breath_rate, motion_level, heart_rate_line, breath_rate_line, motion_contour, motion_levels, axs, heart_rate_text, breathing_rate_text, motion_level_text):
    # Set dynamic x-limits (scrolling effect)
    if len(time_data) > 50:
        axs[0, 0].set_xlim(time_data[-50], time_data[-1])
        axs[0, 1].set_xlim(time_data[-50], time_data[-1])

    # Update heart rate plot
    heart_rate_line.set_data(time_data, heart_rate)

    # Update breathing rate plot
    breath_rate_line.set_data(time_data, breath_rate)

    # Update motion level contour
    motion_levels[:, :-1] = motion_levels[:, 1:]  # Shift data to the left
    motion_levels[:, -1] = motion_level[-1]  # Add new data to the right
    axs[1, 0].cla()  # Clear and redraw the contour plot
    motion_contour = axs[1, 0].contourf(motion_levels, levels=50, cmap="viridis")
    axs[1, 0].set_title("Motion Level Contour")
    axs[1, 0].set_xlabel("Time (s)")
    axs[1, 0].set_ylabel("Spatial Dimension")

    # Update text displays
    if heart_rate:
        heart_rate_text.set_text(f"Heart Rate: {heart_rate[-1]} BPM")
    if breath_rate:
        breathing_rate_text.set_text(f"Breathing Rate: {breath_rate[-1]} BPM")
    if motion_level:
        motion_level_text.set_text(f"Motion Level: {motion_level[-1]}")

    plt.draw()
    plt.pause(0.01)  # Faster updates

# Main function
def main():
    client_socket = connect_to_server()
    
    # Initialize the plots
    fig, heart_rate_line, breath_rate_line, motion_contour, motion_levels, axs, heart_rate_text, breathing_rate_text, motion_level_text = init_plot()

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
                update_plots(time_data, heart_rate_data, breath_rate_data, motion_data, heart_rate_line, breath_rate_line, motion_contour, motion_levels, axs, heart_rate_text, breathing_rate_text, motion_level_text)

        except socket.timeout:
            pass

        # Reduce CPU usage
        time.sleep(0.01)

    client_socket.close()

if __name__ == "__main__":
    main()
