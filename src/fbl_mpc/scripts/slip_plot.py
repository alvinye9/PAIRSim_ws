import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Set font size for various plot components
plt.rcParams.update({
    'axes.titlesize': 20,
    'axes.labelsize': 18,
    'xtick.labelsize': 18,
    'ytick.labelsize': 18,
    'legend.fontsize': 18,
    'font.size': 18
})

class SlipAngleSubscriber(Node):

    def __init__(self):
        super().__init__('slip_angle_subscriber')
        self.front_slip_angle = 0.0
        self.rear_slip_angle = 0.0
        self.max_frt_slip = 0.07
        self.max_rear_slip = 0.08
        self.max_limit = 0.1

        self.front_subscriber = self.create_subscription(
            Float32,
            '/frontslip_angle',
            self.front_slip_angle_callback,
            10)
        
        self.rear_subscriber = self.create_subscription(
            Float32,
            '/rearslip_angle',
            self.rear_slip_angle_callback,
            10)
        
        self.fig, self.ax = plt.subplots()
        self.rects = self.ax.bar(['Front', 'Rear'], [0, 0], color=['blue', 'blue'])
        self.max_rects = self.ax.bar(
            ['Front', 'Rear'], 
            [self.max_frt_slip, self.max_rear_slip], 
            color='none', 
            edgecolor=['grey', 'grey'], 
            linewidth=1.5, 
            alpha=1.0)
        
        self.min_rects = self.ax.bar(
            ['Front', 'Rear'], 
            [-self.max_frt_slip, -self.max_rear_slip], 
            color='none', 
            edgecolor=['grey', 'grey'], 
            linewidth=1.5, 
            alpha=1.0)

        self.ax.set_ylim(-self.max_limit, self.max_limit)
        
        # Customize the plot appearance
        self.ax.set_facecolor('white')
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.spines['left'].set_visible(True)
        self.ax.spines['bottom'].set_visible(True)
        self.ax.yaxis.set_ticks_position('left')
        self.ax.xaxis.set_ticks_position('bottom')
        
        plt.xlabel('Tire')
        plt.ylabel('Slip Angle')
        plt.title('Slip Angle')
        plt.tight_layout()

    def front_slip_angle_callback(self, msg):
        self.front_slip_angle = msg.data
    
    def rear_slip_angle_callback(self, msg):
        self.rear_slip_angle = msg.data

    def update_plot(self, frame):
        self.rects[0].set_height(self.front_slip_angle)
        self.rects[1].set_height(self.rear_slip_angle)
        
        if abs(self.front_slip_angle) <= self.max_frt_slip:
            if abs(self.front_slip_angle) <= 0.035:
                self.rects[0].set_color('green')
            else:
                self.rects[0].set_color('yellow')
        else:
            self.rects[0].set_color('red')
        
        if abs(self.rear_slip_angle) <= self.max_rear_slip:
            if abs(self.rear_slip_angle) <= 0.043:
                self.rects[1].set_color('green')
            else:
                self.rects[1].set_color('yellow')
        else:
            self.rects[1].set_color('red')

def main(args=None):
    rclpy.init(args=args)
    node = SlipAngleSubscriber()

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
