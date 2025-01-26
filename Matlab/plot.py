import pandas as pd
import matplotlib.pyplot as plt

# Load all datasets
adult_data = pd.read_csv('adult_data.csv')
child_data = pd.read_csv('child_data.csv')
cow_data = pd.read_csv('cow_data.csv')
sheep_data = pd.read_csv('sheep_data.csv')
donkey_data = pd.read_csv('donkey_data.csv')
injured_data = pd.read_csv('injured_person_data.csv')

# Extract heart rate and breathing rate
adult_hr = adult_data['HeartRate']
child_hr = child_data['HeartRate']
cow_hr = cow_data['HeartRate']
sheep_hr = sheep_data['HeartRate']
donkey_hr = donkey_data['HeartRate']
injured_hr = injured_data['HeartRate']

adult_br = adult_data['BreathRate']
child_br = child_data['BreathRate']
cow_br = cow_data['BreathRate']
sheep_br = sheep_data['BreathRate']
donkey_br = donkey_data['BreathRate']
injured_br = injured_data['BreathRate']

# Combine the data into matrices for comparison
hr_data = [adult_hr, child_hr, cow_hr, sheep_hr, donkey_hr, injured_hr]
br_data = [adult_br, child_br, cow_br, sheep_br, donkey_br, injured_br]

# Create a figure with two boxplots for HR and BR comparison
plt.figure()

# Boxplot for Heart Rate comparison
plt.subplot(2, 2, 1)
plt.boxplot(hr_data, labels=['Adult', 'Child', 'Cow', 'Sheep', 'Donkey', 'Injured'])
plt.title('Heart Rate Comparison')
plt.ylabel('Heart Rate (bpm)')
plt.grid(True)

# Boxplot for Breathing Rate comparison
plt.subplot(2, 2, 2)
plt.boxplot(br_data, labels=['Adult', 'Child', 'Cow', 'Sheep', 'Donkey', 'Injured'])
plt.title('Breathing Rate Comparison')
plt.ylabel('Breathing Rate (bpm)')
plt.grid(True)

# Overlayed Line Plot for Heart Rate comparison
plt.subplot(2, 2, 3)
plt.plot(adult_hr, label='Adult HR', linewidth=2)
plt.plot(child_hr, label='Child HR', linewidth=2)
plt.plot(cow_hr, label='Cow HR', linewidth=2)
plt.plot(sheep_hr, label='Sheep HR', linewidth=2)
plt.plot(donkey_hr, label='Donkey HR', linewidth=2)
plt.plot(injured_hr, label='Injured HR', linewidth=2)
plt.title('Heart Rate Comparison (Overlayed)')
plt.xlabel('Sample Index')
plt.ylabel('Heart Rate (bpm)')
plt.legend(loc='best')
plt.grid(True)

# Overlayed Line Plot for Breathing Rate comparison
plt.subplot(2, 2, 4)
plt.plot(adult_br, label='Adult BR', linewidth=2)
plt.plot(child_br, label='Child BR', linewidth=2)
plt.plot(cow_br, label='Cow BR', linewidth=2)
plt.plot(sheep_br, label='Sheep BR', linewidth=2)
plt.plot(donkey_br, label='Donkey BR', linewidth=2)
plt.plot(injured_br, label='Injured BR', linewidth=2)
plt.title('Breathing Rate Comparison (Overlayed)')
plt.xlabel('Sample Index')
plt.ylabel('Breathing Rate (bpm)')
plt.legend(loc='best')
plt.grid(True)

# Adjust layout for better presentation
plt.suptitle('Comparison of Heart Rate and Breathing Rate', fontsize=16, fontweight='bold')
plt.tight_layout()
plt.subplots_adjust(top=0.88)

# Display Range for HR and BR
print('Heart Rate Range:')
print(f'Adult: [{adult_hr.min():.2f}, {adult_hr.max():.2f}]')
print(f'Child: [{child_hr.min():.2f}, {child_hr.max():.2f}]')
print(f'Cow: [{cow_hr.min():.2f}, {cow_hr.max():.2f}]')
print(f'Sheep: [{sheep_hr.min():.2f}, {sheep_hr.max():.2f}]')
print(f'Donkey: [{donkey_hr.min():.2f}, {donkey_hr.max():.2f}]')
print(f'Injured: [{injured_hr.min():.2f}, {injured_hr.max():.2f}]')

print('Breathing Rate Range:')
print(f'Adult: [{adult_br.min():.2f}, {adult_br.max():.2f}]')
print(f'Child: [{child_br.min():.2f}, {child_br.max():.2f}]')
print(f'Cow: [{cow_br.min():.2f}, {cow_br.max():.2f}]')
print(f'Sheep: [{sheep_br.min():.2f}, {sheep_br.max():.2f}]')
print(f'Donkey: [{donkey_br.min():.2f}, {donkey_br.max():.2f}]')
print(f'Injured: [{injured_br.min():.2f}, {injured_br.max():.2f}]')

plt.show()
