import rclpy
from rclpy.node import Node
from camera_msgs.msg import RGBD
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import depthai as dai
import os
from datetime import datetime

CWD_PATH = os.path.abspath(os.getcwd())
PATH=CWD_PATH+'/images'
class FrameSaver(Node):
    def __init__(self):
        super().__init__('frame_saver')
        # self.rgbd_subscriber = self.create_subscription(RGBD,'rgbd_frame',self.get_frame,10)
        self.gray_subscriber = self.create_subscription(Image,'oak_pro/left',self.get_gray_frame,1)
        self.depth_subscriber = self.create_subscription(Image,'oak_pro/depth_compressed',self.get_depth_frame,1)
        self.bridge=CvBridge()
        self.depth_frame_copy=None
        # try:
        #     os.mkdir(CWD_PATH+"/images")
        #     os.mkdir(PATH+"/rgb")
        #     os.mkdir(PATH+"/depth")
        # except OSError as err:
        #     self.get_logger().info(err)
        self.get_logger().info(f'Saving images to {PATH}')

    def get_gray_frame(self,gray_msg):
        try:
            curr_epoch_time = gray_msg.header.stamp.sec + gray_msg.header.stamp.nanosec*1e-9
            compressed_data = np.frombuffer(gray_msg.data, dtype=np.uint8)
            gray_frame = cv2.imdecode(compressed_data, cv2.IMREAD_UNCHANGED)

        except CvBridgeError as e1:
            self.get_logger().error("RGB frame CV Bridge failed: "+str(e1))
        if gray_frame is not None:
            gray_filename = "gray"+ str(curr_epoch_time) + ".png"
            gray_filepath = PATH + "/gray"
            cv2.imwrite(os.path.join(gray_filepath,gray_filename),gray_frame)
            self.get_logger().info(f'Saved RGB at {gray_filepath}')
        if cv2.waitKey(1)==ord('q'):
            raise SystemExit

        
    def get_depth_frame(self,depth_msg):
        try:
            curr_epoch_time = depth_msg.header.stamp.sec + depth_msg.header.stamp.nanosec*1e-9
            compressed_data = np.frombuffer(depth_msg.data, dtype=np.uint8)
            # Decode the PNG data to a NumPy array
            depth_frame = cv2.imdecode(compressed_data, cv2.IMREAD_UNCHANGED)
        except CvBridgeError as e2:
            self.get_logger().error("Depth frame CV Bridge failed: "+str(e2))
        if depth_frame is not None:
            depth_filename = "depth_"+str(curr_epoch_time)+".png"
            depth_filepath=PATH+"/depth"
            cv2.imwrite(os.path.join(depth_filepath,depth_filename),depth_frame)
            self.get_logger().info(f'Saved Depth at {depth_filepath}')
        if cv2.waitKey(1)==ord('q'):
            raise SystemExit
    
    def get_frame(self,msg):
        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(msg.rgb,'bgr8')
        except CvBridgeError as e1:
            self.get_logger().error("RGB frame CV Bridge failed: "+str(e1))
        
        try:
            depth_frame = self.bridge.imgmsg_to_cv2(msg.depth,'32FC1')
            self.get_logger().info(f'Depth frame size {np.shape(depth_frame)} {len(np.unique(depth_frame))}')
            self.depth_frame_copy = depth_frame
        except CvBridgeError as e2:
            self.get_logger().error("Depth frame CV Bridge failed: "+str(e2))
        curr_datetime = str(datetime.now())
        if rgb_frame is not None:
            rgb_filename = "rgb_"+curr_datetime+".jpg"
            rgb_filepath = PATH+"/rgb"
            cv2.imwrite(os.path.join(rgb_filepath,rgb_filename),rgb_frame)
            self.get_logger().info(f'Saved RGB at {rgb_filepath}')
        if depth_frame is not None:       
            depth_filename = "depth_"+curr_datetime+".jpg"
            depth_filepath=PATH+"/depth"
            cv2.imwrite(os.path.join(depth_filepath,depth_filename),depth_frame)
            self.get_logger().info(f'Saved Depth at {depth_filepath}')
        if cv2.waitKey(1)==ord('q'):
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    frame_saver = FrameSaver()
    try:
        rclpy.spin(frame_saver)
    except (SystemExit,KeyboardInterrupt):
        rclpy.logging.get_logger("Quitting").info('Done')
    frame_saver.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()
        


        