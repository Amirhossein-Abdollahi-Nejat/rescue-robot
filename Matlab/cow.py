import pandas as pd
import numpy as np

# Set random seed for reproducibility
np.random.seed(42)

# Function to generate realistic data
def generate_data(heart_rate_range, breathing_rate_range, num_samples=50):
    heart_rate = np.random.randint(heart_rate_range[0], heart_rate_range[1] + 1, num_samples)
    breathing_rate = np.random.randint(breathing_rate_range[0], breathing_rate_range[1] + 1, num_samples)
    return pd.DataFrame({'HeartRate': heart_rate, 'BreathRate': breathing_rate})

# Cow data
cow_data = generate_data((80, 85), (46, 50))

# Donkey data
donkey_data = generate_data((57, 60), (26, 28))

# Injured Person data
injured_data = generate_data((97, 100), (27, 30))

# Ship (Human) data
ship_data = generate_data((95, 100), (25, 27))

# Save to CSV files
cow_data.to_csv('cow_data.csv', index=False)
donkey_data.to_csv('donkey_data.csv', index=False)
injured_data.to_csv('injured_person_data.csv', index=False)
ship_data.to_csv('ship_data.csv', index=False)

print("Datasets saved!")
