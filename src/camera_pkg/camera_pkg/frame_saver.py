import rclpy
from rclpy.node import Node
from camera_msgs.msg import RGBD
from sensor_msgs.msg import Image
from message_filters import TimeSynchronizer,Subscriber
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
        self.rgb_subscriber = Subscriber(self,Image,'/oak_pro/left_compressed')
        self.depth_subscriber = Subscriber(self,Image,'/oak_pro/depth_compressed')
        self.sync_subscriber = TimeSynchronizer([self.rgb_subscriber,self.depth_subscriber],10)
        self.sync_subscriber.registerCallback(self.frame_callback)
        self.bridge=CvBridge()
        self.depth_frame_copy=None

        # try:
        #     os.mkdir(CWD_PATH+"/images")
        #     os.mkdir(PATH+"/rgb")
        #     os.mkdir(PATH+"/depth")
        # except OSError as err:
        #     self.get_logger().info(err)

        self.get_logger().info(f'Saving images to {PATH}')

    def frame_callback(self,rgb_msg,depth_msg):
        
        timestamp = rgb_msg.header.stamp 
        # Get grayscale and depth images
        try:
            img_frame = cv2.imdecode(np.frombuffer(rgb_msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
        except CvBridgeError as e1:
            self.get_logger().info(f'RGB frame CV Bridge failed : {e1}')
        try:
            disp_frame = cv2.imdecode(np.frombuffer(depth_msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
            depth_frame = disp_frame/1000 
            # self.get_logger().info(f'DEPTH at {np.shape(depth_frame)[0]//2} {np.shape(depth_frame)[1]//2} == {depth_frame[np.shape(depth_frame)[0]//2][np.shape(depth_frame)[1]//2]}')
        except CvBridgeError as e2:
            self.get_logger().info(f'Depth frame CV Bridge failed : {e2}')
        
        # Save the frames
        curr_epoch_time = timestamp.sec + timestamp.nanosec*1e-9
        if img_frame is not None:
            gray_filename = "gray_"+ str(curr_epoch_time) + ".png"
            gray_filepath = PATH + "/gray"
            cv2.imwrite(os.path.join(gray_filepath,gray_filename),img_frame)
        if cv2.waitKey(1)==ord('q'):
            raise SystemExit
        if depth_frame is not None:
            depth_filename = "depth_"+str(curr_epoch_time)+".png"
            depth_filepath=PATH+"/depth"
            cv2.imwrite(os.path.join(depth_filepath,depth_filename),depth_frame)
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
        


        