import matplotlib.pyplot as plt
import numpy as np
import time
import random

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

    return fig, heart_rate_line, breath_rate_line, motion_line, axs, heart_rate_text, breathing_rate_text, motion_level_text

# Update the plots
def update_plots(time_data, heart_rate, breath_rate, motion_level, heart_rate_line, breath_rate_line, motion_line, axs, heart_rate_text, breathing_rate_text, motion_level_text):
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
    # Initialize the plots
    fig, heart_rate_line, breath_rate_line, motion_line, axs, heart_rate_text, breathing_rate_text, motion_level_text = init_plot()

    heart_rate_data = []
    breath_rate_data = []
    motion_data = []
    time_data = []

    start_time = time.time()

    while True:
        # Simulate heart rate and breathing rate
        heart_rate = random.randint(70, 76)  # Simulated heart rate between 60 and 100 BPM
        breath_rate = random.randint(12, 16)  # Simulated breathing rate between 12 and 20 BPM
        motion_level = random.randint(19, 50)  # Simulated motion level between 0 and 100

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
        update_plots(time_data, heart_rate_data, breath_rate_data, motion_data, heart_rate_line, breath_rate_line, motion_line, axs, heart_rate_text, breathing_rate_text, motion_level_text)

        # Reduce CPU usage
        time.sleep(1)

if __name__ == "__main__":
    main()
