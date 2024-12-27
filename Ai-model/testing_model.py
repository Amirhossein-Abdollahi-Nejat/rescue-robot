# Load the model
model = tf.keras.models.load_model("human_detection_model.h5")

# Example input (replace with real sensor data during runtime)
example_input = np.array([[1.2, 0.8, 78, 18]])  # Distance, Motion Intensity, Heart Rate, Breathing Rate
example_input = scaler.transform(example_input)  # Scale the input

# Predict
prediction = model.predict(example_input)
print("Prediction:", "Human detected" if prediction[0] > 0.5 else "No human detected")
