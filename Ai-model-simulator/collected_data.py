import pandas as pd
import random

def generate_logical_data(num_samples, output_file):
    data = {
        "Distance": [],
        "Motion Intensity": [],
        "Heart Rate": [],
        "Breathing Rate": [],
        "Label": []
    }
    
    for _ in range(num_samples):
        is_human = random.choice([0, 1])  # Randomly decide if it's human or non-human
        
        if is_human:  # Human data
            distance = round(random.uniform(0.5, 2.5), 2)
            motion_intensity = round(random.uniform(50, 100), 2)
            heart_rate = random.randint(60, 100)
            breathing_rate = random.randint(12, 20)
            label = 1
        else:  # Non-human data
            distance = round(random.uniform(2.5, 5.0), 2)
            motion_intensity = round(random.uniform(10, 50), 2)
            heart_rate = random.randint(30, 150) if random.random() > 0.5 else 0
            breathing_rate = random.randint(0, 10) if heart_rate != 0 else 0
            label = 0

        data["Distance"].append(distance)
        data["Motion Intensity"].append(motion_intensity)
        data["Heart Rate"].append(heart_rate)
        data["Breathing Rate"].append(breathing_rate)
        data["Label"].append(label)
    
    # Save the data as a CSV file
    df = pd.DataFrame(data)
    df.to_csv(output_file, index=False)
    print(f"Logical dataset with {num_samples} samples saved to {output_file}")

# Example usage
generate_logical_data(5000, "logical_sensor_data.csv")
