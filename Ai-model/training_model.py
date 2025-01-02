import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sklearn.model_selection import train_test_split
from sklearn.preprocessing import StandardScaler
from sklearn.metrics import accuracy_score
from tensorflow.keras.models import Sequential
from tensorflow.keras.layers import Dense
from tensorflow.keras.models import save_model

# Function to load data from CSV and preprocess it
def load_and_preprocess_data(csv_file):
    # Load data into a pandas dataframe
    df = pd.read_csv(csv_file)

    # Select features (distance, motion, heart rate, breathing rate)
    X = df[["Distance", "Motion Intensity", "Heart Rate", "Breathing Rate"]].values
    y = df["Label"].values  # Labels (1 for human, 0 for non-human)

    # Normalize the features
    scaler = StandardScaler()
    X_scaled = scaler.fit_transform(X)

    return X_scaled, y

# Function to build the neural network model
def build_model(input_dim):
    model = Sequential()
    model.add(Dense(64, input_dim=input_dim, activation='relu'))
    model.add(Dense(32, activation='relu'))
    model.add(Dense(1, activation='sigmoid'))  # Binary output (human or animal)
    model.compile(optimizer='adam', loss='binary_crossentropy', metrics=['accuracy'])
    return model

# Function to train and save the model
def train_and_save_model(csv_file, model_file):
    # Load and preprocess data
    X, y = load_and_preprocess_data(csv_file)

    # Split the dataset into training and testing sets
    X_train, X_test, y_train, y_test = train_test_split(X, y, test_size=0.2, random_state=42)

    # Build the model
    model = build_model(X_train.shape[1])

    # Train the model
    history = model.fit(X_train, y_train, epochs=50, batch_size=32, validation_data=(X_test, y_test))

    # Make predictions and evaluate the model
    y_pred = (model.predict(X_test) > 0.5).astype("int32")
    accuracy = accuracy_score(y_test, y_pred)
    print(f"Model accuracy on the test set: {accuracy * 100:.2f}%")

    # Save the trained model as .h5 file
    model.save(model_file)
    print(f"Model saved to {model_file}")

    # Calculate the predicted probabilities (confidence scores)
    y_pred_prob = model.predict(X_test)

    # Plot training history (Accuracy & Loss)
    plt.figure(figsize=(12, 6))

    # Subplot 1: Accuracy
    plt.subplot(1, 2, 1)
    plt.plot(history.history['accuracy'], label='Training Accuracy')
    plt.plot(history.history['val_accuracy'], label='Validation Accuracy')
    plt.title('Training vs Validation Accuracy')
    plt.xlabel('Epochs')
    plt.ylabel('Accuracy')
    plt.legend()

    # Save Accuracy Plot as PNG
    plt.savefig('training_vs_validation_accuracy.png')
    print("Accuracy plot saved as 'training_vs_validation_accuracy.png'")

    # Subplot 2: Loss
    plt.subplot(1, 2, 2)
    plt.plot(history.history['loss'], label='Training Loss')
    plt.plot(history.history['val_loss'], label='Validation Loss')
    plt.title('Training vs Validation Loss')
    plt.xlabel('Epochs')
    plt.ylabel('Loss')
    plt.legend()

    # Save Loss Plot as PNG
    plt.savefig('training_vs_validation_loss.png')
    print("Loss plot saved as 'training_vs_validation_loss.png'")

    # Show the accuracy and loss plots
    plt.tight_layout()
    plt.show()

    # Plot the Confidence Histogram
    plt.figure(figsize=(6, 4))
    plt.hist(y_pred_prob, bins=20, color='skyblue', edgecolor='black')
    plt.title('Histogram of Model Confidence (Predicted Probabilities)')
    plt.xlabel('Confidence (Probability)')
    plt.ylabel('Frequency')

    # Save Confidence Histogram as PNG
    plt.savefig('confidence_histogram.png')
    print("Confidence histogram saved as 'confidence_histogram.png'")

    # Show the confidence histogram plot
    plt.show()

# Example usage
if __name__ == "__main__":
    csv_file = "sensor_data.csv"  # Replace with the actual CSV file path
    model_file = "human_animal_classifier.h5"
    train_and_save_model(csv_file, model_file)
