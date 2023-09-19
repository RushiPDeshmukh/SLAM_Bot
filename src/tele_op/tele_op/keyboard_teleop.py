import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import keyboard

class keyboard_teleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        self.twist_publisher = self.create_publisher(Twist,'cmd_vel',1)
        self.get_logger().info('Keyboard Teleoperation')
        self.get_logger().info('-   W   -   |   -     Forward     -')
        self.get_logger().info('A   S   D   |  Left     Stop    Right')
        self.get_logger().info('-   x   -   |    -     Backward   -')
        self.get_logger().info('Press any other key to exit.')
    
    def read_key(self):
        cmd = Twist()
        if keyboard.is_pressed('w'): # Forward
            cmd.linear.x = 1 
            cmd.angular.z = 0 
        elif keyboard.is_pressed('a'): # Left
            cmd.linear.x = 0 
            cmd.angular.z = 1
        elif keyboard.is_pressed('s'): # Stop
            cmd.linear.x = 0 
            cmd.angular.z = 0
        elif keyboard.is_pressed('d'): # Right
            cmd.linear.x = 0 
            cmd.angular.z = -1
        elif keyboard.is_pressed('x'): # Backward
            cmd.linear.x = -1 
            cmd.angular.z = 0 
        return cmd
    
    def publish_vel(self):
        while True:
            try:
                cmd_vel = self.read_key()
                self.twist_publisher.publish(cmd_vel)
            except:
                break

def main():
    rclpy.init()
    node = keyboard_teleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
