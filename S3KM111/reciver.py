import socket
import matplotlib.pyplot as plt
import numpy as np

# UDP setup
UDP_IP = "0.0.0.0"  # Listen on all available interfaces
UDP_PORT = 12345     # Same port as Raspberry Pi

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# Radar plot setup
fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111, polar=True)
ax.set_theta_offset(np.pi / 2)  # Rotate the plot to start from the top
ax.set_xlim(0, np.pi / 3)  # Set the 60-degree arc (60 degrees = pi/3 radians)

# Setup empty data arrays
angles = []  # List of angles
distances = []  # List of distances
line, = ax.plot([], [], 'ro')  # Radar plot points (real-time)

# Real-time plot updating function
def update_plot():
    line.set_data(angles, distances)
    plt.draw()
    plt.pause(0.01)  # Pause briefly for the plot to update

# Receive and plot data
try:
    while True:
        data, addr = sock.recvfrom(1024)  # Buffer size 1024
        message = data.decode('utf-8')
        
        if message:
            try:
                # Parse angle and distance
                angle, distance = map(float, message.split(','))

                if 0 <= angle <= 60:  # Filter angles in the 60-degree range
                    angles.append(np.radians(angle))  # Convert angle to radians for polar plot
                    distances.append(distance)  # Add the distance value

                    # Update the plot with the new data
                    update_plot()

            except Exception as e:
                print(f"Error parsing data: {e}")

except KeyboardInterrupt:
    print("Exiting...")
finally:
    sock.close()
    plt.close()
