import rclpy
from rclpy.node import Node
from camera_msgs.msg import RGBD
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import depthai as dai

class RGBDSubscriber(Node):
    def __init__(self):
        super().__init__('rgbd_subscriber')
        self.rgbd_subscriber = self.create_subscription(RGBD,'rgbd_frame',self.get_frame,10)
        self.bridge=CvBridge()
        self.depth_frame_copy=None

    def get_frame(self,msg):
        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(msg.rgb,'bgr8')
        except CvBridgeError as e1:
            self.get_logger().error("RGB frame CV Bridge failed: "+str(e1))
        
        try:
            depth_frame = self.bridge.imgmsg_to_cv2(msg.depth,'32FC1')
            self.depth_frame_copy = depth_frame
        except CvBridgeError as e2:
            self.get_logger().error("Depth frame CV Bridge failed: "+str(e2))
        
        if rgb_frame is not None:
            cv2.namedWindow("rgb",cv2.WINDOW_NORMAL)
            cv2.imshow("rgb",rgb_frame)
        if depth_frame is not None:       
            cv2.namedWindow("depth",cv2.WINDOW_NORMAL)
            cv2.setMouseCallback("depth", self.on_mouse_click)
            cv2.imshow("depth",depth_frame*100)
            

            # y=652
            # self.get_logger().info(f'Depth of x: {x}, y:{y} == {(depth_frame[y][x])}')
        if cv2.waitKey(1)==ord('q'):
            raise SystemExit
        
    def on_mouse_click(self, event, x, y, flags, param):
        """
        Callback function for mouse click events.

        Args:
        - event: The type of mouse event (e.g., cv2.EVENT_LBUTTONDOWN).
        - x: The x-coordinate of the mouse click.
        - y: The y-coordinate of the mouse click.
        - flags: Additional flags (not used).
        - param: Additional parameters (not used).

        Returns:
        - None
        """
        if event == cv2.EVENT_LBUTTONDOWN:
            # Print the depth value at the clicked point
            depth_value = self.depth_frame_copy[y][x]
            self.get_logger().info(f"Depth value at point ({x}, {y}): {depth_value}")

def main(args=None):
    rclpy.init(args=args)
    rgbd_subscriber = RGBDSubscriber()
    try:
        rclpy.spin(rgbd_subscriber)
    except (SystemExit,KeyboardInterrupt):
        rclpy.logging.get_logger("Quitting").info('Done')
    rgbd_subscriber.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
        


        