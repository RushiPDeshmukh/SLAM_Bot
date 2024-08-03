import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class RGBDepthSubscriber(Node):
    def __init__(self):
        super().__init__('rgb_depth_subscriber')
        self.rgb_subscriber = self.create_subscription(Image,'rgb_frame',self.get_rgb_frame,10)
        self.depth_subscriber = self.create_subscription(Image,'depth_frame',self.get_depth_frame,10)

        self.prev_rgb_time = self.get_clock().now().nanoseconds
        self.prev_depth_time = self.get_clock().now().nanoseconds
        
    def get_rgb_frame(self,rgb_msg):
        this_rgb_time = rgb_msg.header.stamp.sec + (rgb_msg.header.stamp.nanosec*1e-9)
        self.get_logger().info(f'RGB message freq {((this_rgb_time-self.prev_rgb_time)*1e-9)}')
        self.prev_rgb_time = this_rgb_time

    def get_depth_frame(self,depth_msg):
        this_depth_time = depth_msg.header.stamp.sec + (depth_msg.header.stamp.nanosec*1e-9)
        self.get_logger().info(f'Depth message freq {((this_depth_time-self.prev_depth_time)*1e-9)}')
        self.prev_depth_time = this_depth_time

def main(args=None):
    rclpy.init(args=args)
    rgbd_sub = RGBDepthSubscriber()
    try:
        rclpy.spin(rgbd_sub)
    except (SystemExit,KeyboardInterrupt):
        rclpy.logging.get_logger("Quitting").info('Done')
    rgbd_sub.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
    
