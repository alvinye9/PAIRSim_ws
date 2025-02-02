import numpy as np
import matplotlib.pyplot as plt
from bicycle_model import Bicycle_model
# Assuming the Bicycle_model class is already defined and imported

# Dummy node class to simulate parameter declaration and retrieval
class Node:
    def __init__(self):
        self.params = {}
    
    def declare_parameter(self, name, value):
        self.params[name] = value
    
    def get_parameter(self, name):
        return self.params[name]

# Initialize the node and bicycle model
node = Node()
horizon = 10  # Define the horizon for future inputs
dt = 0.1  # Define the time step
bicycle = Bicycle_model(node, horizon, dt)

# Set initial conditions
initial_speed = 10.0  # m/s
initial_roll_angle = 0.0
initial_v_long = initial_speed
initial_v_lat = 0.0
initial_w = 0.0
initial_steering_angle = 10.0  # positive steering angle in degrees
future_steering_angle = np.full(horizon, initial_steering_angle)  # constant steering input
future_acc_long = np.zeros(horizon)  # zero acceleration

# Update the bicycle model with initial conditions and future inputs
bicycle.update_current_state(
    roll_angle=initial_roll_angle,
    current_v_long=initial_v_long,
    current_v_lat=initial_v_lat,
    current_w=initial_w,
    current_steering_angle=initial_steering_angle,
    future_steering_angle=future_steering_angle,
    future_acc_long=future_acc_long
)

# Calculate future states
positions, v_longs, v_lats, yaw_rates = [], [], [], []
total_time = bicycle.horizon * bicycle.mpc_time_step
num_samples = int(total_time / bicycle.sample_t) + 1
times = np.linspace(0, total_time, num=num_samples)

position = np.array([0.0, 0.0, 0.0])
v_long = bicycle.current_v_long
v_lat = bicycle.current_v_lat
w = bicycle.current_w

positions.append(position)
v_longs.append(v_long)
v_lats.append(v_lat)
yaw_rates.append(w)

for i in range(1, len(times)):
    next_position, w, v_long, v_lat, w = bicycle.calculate_next_position(
        bicycle.bank_angle, v_long, v_lat, future_acc_long[i % horizon], w, future_steering_angle[i % horizon], position
    )
    positions.append(next_position)
    v_longs.append(v_long)
    v_lats.append(v_lat)
    yaw_rates.append(w)
    position = next_position

# Convert lists to arrays for plotting
positions = np.array(positions)
v_longs = np.array(v_longs)
v_lats = np.array(v_lats)
yaw_rates = np.array(yaw_rates)

# Plot lateral velocity vs. time
plt.figure(figsize=(10, 6))
plt.plot(times, v_lats, label='Lateral Velocity')
plt.xlabel('Time (s)')
plt.ylabel('Lateral Velocity (m/s)')
plt.title('Lateral Velocity vs. Time')
plt.legend()
plt.grid()
plt.show()
