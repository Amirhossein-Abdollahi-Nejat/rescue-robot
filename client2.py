import socket
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# Set up socket client
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.1.32', 12345))  # Replace with your Pi's IP address

# Create a figure with two sections: left for text and right for radar plot
fig = plt.figure(figsize=(10, 5))
gs = fig.add_gridspec(1, 2, width_ratios=[1, 2])  # 1:2 ratio for text and plot

# Left side for text
text_ax = fig.add_subplot(gs[0])
text_ax.axis('off')  # Hide axes
distance_text = text_ax.text(0.5, 0.8, '', color='blue', fontsize=14, ha='center')
x_text = text_ax.text(0.5, 0.6, '', color='green', fontsize=14, ha='center')
y_text = text_ax.text(0.5, 0.4, '', color='red', fontsize=14, ha='center')

# Right side for radar plot
plot_ax = fig.add_subplot(gs[1], projection='polar')
plot_ax.set_xlim(0, 2 * np.pi)  # Angle range (0 to 360 degrees)
plot_ax.set_ylim(0, 4)  # Distance range (adjust as per your sensor's range)
point, = plot_ax.plot([], [], 'ro')  # Red point for the current position

def update(frame):
    """Update the radar plot and display distance, x, y values."""
    try:
        data = client_socket.recv(1024).decode().strip()
        if data:
            x, y = map(float, data.split(','))

            # Convert Cartesian (x, y) to Polar (r, θ)
            r = np.sqrt(x**2 + y**2)
            theta = np.arctan2(y, x)
            if theta < 0:  # Ensure θ is in the range [0, 2π]
                theta += 2 * np.pi
            
            # Update radar point
            point.set_data([theta], [r])

            # Update text values
            distance_text.set_text(f"Distance: {r:.2f} m")
            x_text.set_text(f"X: {x:.2f} m")
            y_text.set_text(f"Y: {y:.2f} m")
    except Exception as e:
        print(f"Error: {e}")
        client_socket.close()
        exit()

    return point, distance_text, x_text, y_text

# Create animation for real-time radar plot
ani = FuncAnimation(fig, update, blit=True, interval=100)
plt.tight_layout()
plt.show()

client_socket.close()
