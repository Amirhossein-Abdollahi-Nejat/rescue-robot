# Rescue Robot - Human Detection AI

## Overview

This repository contains the code for a **Rescue Robot** equipped with multiple sensors to aid in human detection and search and rescue operations. The robot uses a **Deep Learning** model to analyze sensor data and classify whether an object is a **human** or **non-human**.

## AI Model

The **Human Detection AI** is built using **Deep Learning** techniques, specifically a **Multi-Layer Perceptron (MLP)**. The model takes the following input features from the sensors:

- **Distance** (from an ultrasonic sensor)
- **Motion Intensity** (detected by a motion sensor)
- **Heart Rate** (detected by a specialized sensor, e.g., MR60BHA1)
- **Breathing Rate** (detected by a specialized sensor, e.g., MR60BHA1)

Based on this data, the model classifies the object into one of two categories:

- **Human (1)**
- **Non-human (0)**

The model was trained using labeled data collected by the robot’s sensors. This trained model is saved in the **`human_detection_model.h5`** file, which can be used in real-time for detecting human presence.

## Additional Sensors Used

In addition to the heart rate and breathing rate sensors, the robot also uses:

- **Ultrasonic Sensors** for distance measurement to detect obstacles or human presence.
- **Motion Sensors** (e.g., SW-18010P) for detecting any motion in the environment.

These sensors work together to gather a variety of data points, which are fed into the AI model for classification. The combination of sensor data ensures accurate human detection, even in challenging environments.

## Conclusion

The AI model, coupled with the robot's diverse sensor suite, enables real-time human detection, making it a critical tool for search and rescue operations in disaster scenarios.
