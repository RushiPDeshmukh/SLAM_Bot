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

    def get_frame(self,msg):
        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(msg.rgb,'bgr8')
        except CvBridgeError as e1:
            self.get_logger().error("RGB frame CV Bridge failed: "+str(e1))
        
        try:
            depth_frame = self.bridge.imgmsg_to_cv2(msg.depth,'8UC1')
        except CvBridgeError as e2:
            self.get_logger().error("Depth frame CV Bridge failed: "+str(e2))
        
        if rgb_frame is not None:
            cv2.namedWindow("rgb",cv2.WINDOW_NORMAL)
            cv2.imshow("rgb",rgb_frame)
        if depth_frame is not None:       
            cv2.namedWindow("depth",cv2.WINDOW_NORMAL)
            cv2.imshow("depth",depth_frame)

        if cv2.waitKey(1)==ord('q'):
            raise SystemExit
        
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
        


        