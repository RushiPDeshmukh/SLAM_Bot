import rclpy
from rclpy.node import Node
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu
from matplotlib.animation import FuncAnimation

class IMU_Plot(Node):
    def __init__(self):
        super().__init__('live_plot_node')
        self.raw_data_subscriber = self.create_subscription(Imu,'rp2040_imu_info_topic',self.raw_data_callback,10)

        # Initialize the plot
        self.fig, self.ax = plt.subplots()
        self.x_data, self.y_data = [], []
        self.line, = self.ax.plot(self.x_data, self.y_data, label='IMU')
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value')
        self.ax.legend()

    def callback(self, msg):
        # Update the plot with new data
        self.x_data.append(self.x_data[-1] + 1 if self.x_data else 0)  # Incremental x-axis
        self.y_data.append(msg.data)

        # Keep a fixed number of points on the plot (adjust as needed)
        max_points = 100
        if len(self.x_data) > max_points:
            self.x_data = self.x_data[-max_points:]
            self.y_data = self.y_data[-max_points:]

        # Update the plot data
        self.line.set_data(self.x_data, self.y_data)

    def run(self):
        # Set up the animation
        ani = FuncAnimation(self.fig, self.animate, interval=100)
        plt.show()

    def animate(self, frame):
        # Animation function (used by FuncAnimation)
        pass  # This function is empty since the plot is updated in the callback


def main():
    rclpy.init()
    node = IMU_Plot()
    node.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
