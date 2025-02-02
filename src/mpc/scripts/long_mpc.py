#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32
import numpy as np
from scipy.interpolate import interp1d
from scipy.linalg import lstsq
import cvxpy as cp

class VelocitySubscriber(Node):

    def __init__(self):
        super().__init__('velocity_subscriber')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            '/velocities',
            self.listener_callback,
            10)
        self.subscription = self.create_subscription(
            Float32,
            '/localization/vehicle_speed',
            self.speed_callback,
            10)
        self.curv_subscription = self.create_subscription(
            Float32MultiArray,
            '/planning/front_path/curvature',
            self.curv_callback,
            10
        )
        self.accel_pub = self.create_publisher(Float32, '/control/desired_acceleration', 10)
        self.subscription  # prevent unused variable warning
        timer_period = 0.01
        self.timer = self.create_timer(timer_period, self.simple_mpc)

        self.velocity_array = []
        self.curv_array = []
        self.curr_speed = 0

    def listener_callback(self, msg):
        self.velocity_array = msg.data  # This is a list of float32

    def speed_callback(self, msg):
        self.curr_speed = msg.data
    
    def curv_callback(self, msg):
        self.curv_array = msg.data

    def spatial_to_temporal_velocity(self, v_spatial, curv_spatial, d, N, dt):
        # v_spatial: Array of desired velocities at each spatial point (spatially distributed every 'd' meters)
        # d: The spatial distance between points in meters
        # N: Number of timesteps in the prediction horizon
        # dt: Time between each timestep in the temporal control horizon

        # Calculate the average velocity for each spatial segment
        v_spatial = np.array(v_spatial)
        v_avg = (v_spatial[:-1] + v_spatial[1:]) / 2
        #print(v_avg)

        # Calculate the time to travel 'd' meters for each segment
        t_segment = d / v_avg

        # Calculate the cumulative time for each spatial velocity reference
        t_cumulative = np.cumsum(t_segment)
        t_cumulative = np.insert(t_cumulative, 0, 0)  # Insert 0 at the beginning to start the profile
        #print(t_cumulative)
        #print(len(v_spatial))

        # Define the control system's time steps as a time horizon
        t_control = np.arange(0, N * dt, dt)

        # Interpolate the spatial velocities to the control system's time steps
        interp_func = interp1d(t_cumulative, v_spatial, kind='linear', bounds_error=False, fill_value='extrapolate')

        # Get the desired velocity at each time step
        v_temporal = interp_func(t_control)
        
        interp_func = interp1d(t_cumulative, curv_spatial, kind='linear', bounds_error=False, fill_value='extrapolate')
        curv_temporal = interp_func(t_control)

        return v_temporal, abs(curv_temporal)

    def simple_mpc(self, N=50, dt=0.1, G=2.0, R=10.0, Q=0.7):
        # N: Number of timesteps in the prediction horizon
        # dt: Time between each timestep
        # R: Weight on the acceleration squared term in the cost function
        if len(self.velocity_array) == 0 or len(self.curv_array) == 0:
            return
        # Handle zeros in the velocity profile (replacing with smallest positive float)
        v_desired, curv = self.spatial_to_temporal_velocity(self.velocity_array, self.curv_array, 2.0, N, dt)
        v_current = self.curr_speed
        
        # Define the control variable (acceleration at each timestep)
        a = cp.Variable(N)

        # Define the state variable (velocity at each timestep)
        v = cp.Variable(N+1)
        #print(v_desired-v_current)
        
        # Initial state constraint
        constraints = [v[0] == v_current]
        
        # Dynamics constraints (for each timestep)
        for t in range(N):
            constraints.append(v[t+1] == v[t] + a[t]*dt)
            constraints.append(a[t] <= 15.0)
            constraints.append(a[t] >= -15.0)



        # Objective function
        min_des = min(v_desired)
        objective = cp.Minimize(G * cp.sum_squares(v[1:] - v_desired[:N]) + Q * cp.sum_squares(min_des - v[1:]) + R*cp.sum_squares(a))

        # Define the problem and solve it
        problem = cp.Problem(objective, constraints)
        problem.solve(solver=cp.OSQP, verbose=False)


        # Check if the problem is solved successfully
        if problem.status not in ["infeasible", "unbounded"]:
            # Assuming the problem is solved, the optimal acceleration sequence is:
            a_optimal = a.value
            v_optimal = v.value
        else:
            # In case the problem is infeasible or unbounded, we may need to return some default values
            a_optimal = np.zeros(N)
            v_optimal = np.full(N+1, v_current)
        #print(f"Desired velocity: {self.velocity_array[0]}, Optimal Velocity: {v_optimal[0]}, optimal accel: {a_optimal[0]}")

        msg = Float32()
        msg.data = a_optimal[0]
        self.accel_pub.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)

    velocity_subscriber = VelocitySubscriber()

    try:
        rclpy.spin(velocity_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        velocity_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
