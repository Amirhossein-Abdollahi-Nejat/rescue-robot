import serial
import csv
import time

# Configure the serial port (adjust based on your setup)
SERIAL_PORT = "/dev/ttyUSB0"  # Replace with your actual serial port
BAUD_RATE = 115200

# CSV file to save data
FILE_NAME = "sensor_data.csv"
FIELDS = ["Timestamp", "Distance", "Motion Intensity", "Heart Rate", "Breathing Rate", "Label"]

# Function to collect data from the sensor for a specified duration (in minutes)
def collect_and_log_data(label, duration_minutes):
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

                except KeyboardInterrupt:
                    print("Data collection stopped.")
                    break
                except Exception as e:
                    print(f"Error during data collection: {e}")
                    continue

            print(f"Data collection for {duration_minutes} minutes completed.")

    except Exception as e:
        print(f"Error setting up serial connection: {e}")

# Example usage
label = input("Enter label (1 for human, 0 for non-human): ")
duration_minutes = int(input("Enter the duration in minutes for data collection: "))
collect_and_log_data(label, duration_minutes)

print("Data collection process completed.")


###########################################################
# OUTPUT SIMULATION FOR Ai-model/data_collection.py


#   Enter label (1 for human, 0 for non-human): 1
#   Enter the duration in minutes for data collection: 5
#
#   Collecting data with label: 1 for 5 minutes...
#
#   Logged data: 0.52, 0.78, 70, 18, Label: 1
#   Logged data: 0.53, 0.80, 71, 19, Label: 1
#   Logged data: 0.51, 0.79, 72, 18, Label: 1
#   Logged data: 0.54, 0.82, 73, 20, Label: 1
#   ...
#   Data collection for 5 minutes completed.
#   Data collection process completed.

###########################################################

