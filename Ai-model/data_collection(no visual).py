import serial
import csv
import time

# Configure the serial port (adjust based on your setup)
SERIAL_PORT = "/dev/ttyUSB0"  # Replace with your actual serial port
BAUD_RATE = 115200

# CSV file to save data
FILE_NAME = "sensor_data.csv"
FIELDS = ["Timestamp", "Distance", "Motion Intensity", "Heart Rate", "Breathing Rate", "Label"]

# Function to initialize the serial connection
def initialize_serial_connection():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        return ser
    except Exception as e:
        print(f"Error setting up serial connection: {e}")
        return None

# Function to write data to CSV
def write_to_csv(file, data):
    try:
        writer = csv.writer(file)
        writer.writerow(data)
    except Exception as e:
        print(f"Error writing to CSV: {e}")

# Function to process a single sensor data line
def process_data_line(data_line, label):
    values = data_line.split(",")  # Adjust parsing based on your sensor's data format
    if len(values) >= 4:
        timestamp = time.time()
        distance, motion, heart_rate, breathing_rate = map(float, values[:4])
        return [timestamp, distance, motion, heart_rate, breathing_rate, label]
    return None

# Function to collect data from the sensor and log it
def collect_and_log_data(label, num_samples):
    print(f"Collecting {num_samples} samples with label: {label}...")
    ser = initialize_serial_connection()
    if not ser:
        return

    try:
        # Open the CSV file in append mode
        with open(FILE_NAME, mode="a", newline="") as file:
            writer = csv.writer(file)

            # Write header if the file is empty
            if file.tell() == 0:
                writer.writerow(FIELDS)

            sample_count = 0
            while sample_count < num_samples:
                try:
                    # Read and process the sensor data from the serial port
                    data_line = ser.readline().decode("utf-8").strip()
                    processed_data = process_data_line(data_line, label)
                    
                    if processed_data:
                        # Write the processed data to the CSV file
                        write_to_csv(file, processed_data)
                        print(f"Logged data: {processed_data[1]}, {processed_data[2]}, {processed_data[3]}, {processed_data[4]}, Label: {label}")

                        sample_count += 1
                        print(f"Sample {sample_count}/{num_samples} collected.")
                    else:
                        print("Invalid data received.")

                except KeyboardInterrupt:
                    print("Data collection stopped.")
                    break
                except Exception as e:
                    print(f"Error during data collection: {e}")
                    continue

            print(f"Data collection for {num_samples} samples completed.")

    except Exception as e:
        print(f"Error during data collection: {e}")
    finally:
        ser.close()  # Ensure the serial connection is closed when done

# Example usage
def main():
    label = input("Enter label (1 for human, 0 for non-human): ")
    num_samples = int(input("Enter the number of samples for data collection: "))
    collect_and_log_data(label, num_samples)
    print("Data collection process completed.")

if __name__ == "__main__":
    main()

# Sample Console Output (This part is commented out for reference)
# -------------------------------------------------------------
# Enter label (1 for human, 0 for non-human): 1
# Enter the number of samples for data collection: 5
# Collecting 5 samples with label: 1...
# Logged data: 2.34, 0.75, 68.0, 15.0, Label: 1
# Sample 1/5 collected.
# Logged data: 2.29, 0.80, 69.0, 16.0, Label: 1
# Sample 2/5 collected.
# Logged data: 2.30, 0.78, 70.0, 14.5, Label: 1
# Sample 3/5 collected.
# Logged data: 2.32, 0.82, 68.5, 15.5, Label: 1
# Sample 4/5 collected.
# Logged data: 2.33, 0.77, 69.5, 15.0, Label: 1
# Sample 5/5 collected.
# Data collection for 5 samples completed.
