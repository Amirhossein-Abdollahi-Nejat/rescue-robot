import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import math

class RadarSimulation:
    def __init__(self):
        # Setup plot with dark background
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        
        # Set plot limits and labels
        self.ax.set_xlim(0, 10)
        self.ax.set_ylim(-5, 5)
        self.ax.set_xlabel('Distance along longitudinal axis (m)')
        self.ax.set_ylabel('Distance along lateral axis (m)')
        self.ax.set_title('Radar Detection Simulation')
        
        # Add grid
        self.ax.grid(True, linestyle='--', alpha=0.3)
        
        # Add range circles
        circles = [2, 4, 6, 8, 10]
        for radius in circles:
            circle = plt.Circle((0, 0), radius, fill=False, color='green', alpha=0.3)
            self.ax.add_artist(circle)
        
        # Initialize scatter plot for person detection
        self.scatter = self.ax.scatter([], [], c='red', s=100)
        
        # Simulated person movement
        self.t = 0
        
        # Add radar position indicator
        self.ax.scatter(0, 0, c='green', marker='^', s=100, label='Radar')
        self.ax.legend()

    def simulate_person_movement(self):
        """Simulate a person walking in a pattern"""
        # Create a figure-8 pattern
        x = 3 + 2 * math.cos(self.t * 0.1)  # Person moves between 1-5m in x
        y = 2 * math.sin(self.t * 0.2)      # Person moves between -2 and 2m in y
        
        # Calculate straight-line distance
        distance = math.sqrt(x**2 + y**2)
        
        self.t += 1
        return x, y, distance

    def update(self, frame):
        # Get simulated position
        x, y, distance = self.simulate_person_movement()
        
        # Update scatter plot
        self.scatter.set_offsets([[x, y]])
        
        # Print position data
        print(f"Distance: {distance:.2f}m, X: {x:.2f}m, Y: {y:.2f}m")
        
        return self.scatter,

    def start_simulation(self):
        ani = FuncAnimation(self.fig, self.update, interval=100, blit=True)
        plt.show()

if __name__ == "__main__":
    try:
        sim = RadarSimulation()
        print("Starting radar simulation...")
        print("Press Ctrl+C to stop")
        sim.start_simulation()
    except KeyboardInterrupt:
        print("\nStopping simulation...")
    finally:
        plt.close()