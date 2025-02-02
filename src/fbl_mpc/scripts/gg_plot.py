import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class AccelerationSubscriber(Node):

    def __init__(self):
        super().__init__('acceleration_subscriber')
        self.g = 9.81  # Acceleration due to gravity in m/s^2

        # Circular buffer to store the last 2000 points
        self.buffer_size = 400
        self.lat_acc_buffer = np.zeros(self.buffer_size)
        self.long_acc_buffer = np.zeros(self.buffer_size)
        self.index = 0
        self.full = False  # Flag to indicate if the buffer is full

        # Subscription to the IMU topic
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/novatel_bottom/imu/data',
            self.acc_v_cb,
            10
        )

        # Initialize the plot
        plt.rcParams.update({'font.size': 18})  # Set global font size

        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'b-')
        self.latest_point, = self.ax.plot([], [], 'ro', markersize=10)  # Red dot for the latest point
        self.ax.set_xlim(-3 * self.g-5, 3 * self.g+5)
        self.ax.set_ylim(-3 * self.g, 3 * self.g)

        # Draw circles for 1g, 2g, 3g with thicker lines
        self.circles = [plt.Circle((0, 0), radius=g * self.g, color=color, fill=False, linewidth=2, label=f'{g}g') 
                        for g, color in zip([1, 2, 3], ['red', 'green', 'blue'])]
        for circle in self.circles:
            self.ax.add_artist(circle)

        # Add grid, labels, title, and legend
        plt.grid(True)
        plt.xlabel('Lateral Acceleration (m/s²)')
        plt.ylabel('Longitudinal Acceleration (m/s²)')
        plt.title('g-g')
        plt.legend()
        plt.tight_layout()

    def acc_v_cb(self, msg):
        # Update the circular buffer with new data
        self.lat_acc_buffer[self.index] = msg.linear_acceleration.y
        self.long_acc_buffer[self.index] = msg.linear_acceleration.x
        self.index = (self.index + 1) % self.buffer_size
        if self.index == 0:
            self.full = True

    def update_plot(self, frame):
        if self.full:
            x_data = np.concatenate((self.lat_acc_buffer[self.index:], self.lat_acc_buffer[:self.index]))
            y_data = np.concatenate((self.long_acc_buffer[self.index:], self.long_acc_buffer[:self.index]))
        else:
            x_data = self.lat_acc_buffer[:self.index]
            y_data = self.long_acc_buffer[:self.index]

        self.line.set_data(x_data, y_data)
        if len(x_data) > 0 and len(y_data) > 0:  # Ensure there is data before updating the latest point
            self.latest_point.set_data(x_data[-1], y_data[-1])  # Update the latest point
        self.ax.set_aspect('equal', 'box')  # Ensure the plot window is always square
        return self.line, self.latest_point

def main(args=None):
    rclpy.init(args=args)
    node = AccelerationSubscriber()

    # Use FuncAnimation to call the update_plot function
    ani = FuncAnimation(node.fig, node.update_plot, interval=100)
    
    try:
        import threading
        ros_thread = threading.Thread(target=rclpy.spin, args=(node,))
        ros_thread.start()
        
        plt.show()
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        ros_thread.join()

if __name__ == '__main__':
    main()
