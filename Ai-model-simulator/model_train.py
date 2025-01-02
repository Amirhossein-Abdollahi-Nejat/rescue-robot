import pandas as pd
import numpy as np
import tensorflow as tf
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.optimizers import Adam
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler

# Load the dataset
df = pd.read_csv("logical_sensor_data.csv")

# Split data into features (X) and labels (y)
X = df.drop(columns=["Label"])
y = df["Label"]

# Normalize the features
scaler = StandardScaler()
X_scaled = scaler.fit_transform(X)

# Split the data into training and testing sets
X_train, X_test, y_train, y_test = train_test_split(X_scaled, y, test_size=0.2, random_state=42)

# Create the model
model = Sequential()

# Input layer and first hidden layer with SELU activation
model.add(Dense(64, input_dim=X_train.shape[1], activation="selu"))

# Second hidden layer with SELU activation
model.add(Dense(64, activation="selu"))

# Third hidden layer with tanh activation
model.add(Dense(64, activation="tanh"))

# Fourth hidden layer with SELU activation
model.add(Dense(64, activation="selu"))

# Output layer with sigmoid activation for binary classification
model.add(Dense(1, activation="sigmoid"))

# Compile the model
model.compile(optimizer=Adam(), loss="binary_crossentropy", metrics=["accuracy"])

# Train the model
history = model.fit(X_train, y_train, epochs=50, batch_size=32, validation_data=(X_test, y_test))

# Save the trained model as an .h5 file
model.save("logical_sensor_model.h5")
print("Model saved to logical_sensor_model.h5")

# Plot the training history
# Accuracy plot
plt.figure(figsize=(12, 6))
plt.subplot(1, 2, 1)
plt.plot(history.history["accuracy"], label="Train Accuracy")
plt.plot(history.history["val_accuracy"], label="Test Accuracy")
plt.title("Model Accuracy")
plt.xlabel("Epochs")
plt.ylabel("Accuracy")
plt.legend()

# Loss plot
plt.subplot(1, 2, 2)
plt.plot(history.history["loss"], label="Train Loss")
plt.plot(history.history["val_loss"], label="Test Loss")
plt.title("Model Loss")
plt.xlabel("Epochs")
plt.ylabel("Loss")
plt.legend()

# Save the plots as images
plt.tight_layout()
plt.savefig("training_performance.png")
plt.show()

print("Training performance plot saved as 'training_performance.png'")
