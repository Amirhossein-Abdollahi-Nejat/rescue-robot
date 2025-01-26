import socket
import matplotlib.pyplot as plt
import numpy as np
import struct

# Raspberry Pi IP and PORT
RPI_IP = "192.168.1.107"  # Replace with actual Raspberry Pi IP
PORT = 9001

# Create a TCP client
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((RPI_IP, PORT))
print("Connected to Raspberry Pi!")

# Initialize the live plot
plt.ion()  # Interactive mode ON
fig, ax = plt.subplots()
x_data = np.arange(100)  # Store 100 previous data points
y_data = np.zeros(100)   # Initial zero values
line, = ax.plot(x_data, y_data, '-r')  # Red line for real-time data
ax.set_ylim(-1000, 1000)  # Adjust Y-axis range based on expected values

while True:
    try:
        raw_data = client.recv(4)  # Receive 4 bytes at a time (float or int)
        if len(raw_data) == 4:
            value = struct.unpack('<f', raw_data)[0]  # Convert bytes to float
            print(f"Received: {value}")

            # Update the plot
            y_data = np.roll(y_data, -1)  # Shift data left
            y_data[-1] = value  # Add new data at the end
            line.set_ydata(y_data)  # Update line data

            plt.draw()
            plt.pause(0.01)  # Small pause for real-time update

    except KeyboardInterrupt:
        print("Closing connection...")
        break

client.close()
plt.ioff()  # Turn off interactive mode
plt.show()  # Show final plot
