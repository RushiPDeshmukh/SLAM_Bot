import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

class ImuPlotNode(Node):
    def __init__(self):
        super().__init__('imu_plot_node')
        self.raw_imu_subscription = self.create_subscription(
            Imu,
            'rp2040_imu_raw',  # Replace with your actual raw IMU topic
            self.raw_imu_callback,
            10
        )
        self.raw_imu_subscription  # Prevent unused variable warning

        self.calibrated_imu_subscription = self.create_subscription(
            Imu,
            'imu',  # Replace with your actual calibrated IMU topic
            self.calibrated_imu_callback,
            10
        )
        self.calibrated_imu_subscription  # Prevent unused variable warning

        self.fig, self.ax = plt.subplots()
        self.time_data = []
        self.raw_data = {'x': [], 'y': [], 'z': []}
        self.calibrated_data = {'x': [], 'y': [], 'z': []}

        self.line_raw, = self.ax.plot([], [], label='Raw IMU')
        self.line_calibrated, = self.ax.plot([], [], label='Calibrated IMU')

        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Acceleration')
        self.ax.legend()

    def raw_imu_callback(self, msg):
        self.time_data.append(self.get_clock().now().to_msg().sec)  # Use timestamp as x-axis data
        self.raw_data['x'].append(msg.linear_acceleration.x)
        self.raw_data['y'].append(msg.linear_acceleration.y)
        self.raw_data['z'].append(msg.linear_acceleration.z)

        self.update_plot()

    def calibrated_imu_callback(self, msg):
        self.calibrated_data['x'].append(msg.linear_acceleration.x)
        self.calibrated_data['y'].append(msg.linear_acceleration.y)
        self.calibrated_data['z'].append(msg.linear_acceleration.z)

        self.update_plot()

    def update_plot(self):
        max_points = 100  # Adjust the number of points displayed on the plot

        if len(self.time_data) > max_points:
            self.time_data = self.time_data[-max_points:]
            for axis in ['x', 'y', 'z']:
                self.raw_data[axis] = self.raw_data[axis][-max_points:]
                self.calibrated_data[axis] = self.calibrated_data[axis][-max_points:]

        self.line_raw.set_data(self.time_data, self.raw_data['x'])  # Display data for the 'x' axis
        self.line_calibrated.set_data(self.time_data, self.calibrated_data['x'])

        self.ax.relim()
        self.ax.autoscale_view()

def main():
    rclpy.init()
    node = ImuPlotNode()
    ani = FuncAnimation(node.fig, node.update_plot, interval=100)
    plt.show()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
