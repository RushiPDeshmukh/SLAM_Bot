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
        self.gray_subscriber = self.create_subscription(Image,'oak_pro/left_compressed',self.get_gray_frame,1)
        self.depth_subscriber = self.create_subscription(Image,'oak_pro/depth_compressed',self.get_depth_frame,1)
        self.bridge=CvBridge()
        self.depth_frame_copy=None

        """ Oak D Pro config """
        image_width = 640 # 400P setting for mono camera & stereo is scaled to mono
        horizontal_fov = 80 # deg
        self.baseline = 0.075 # m
        self.focal_length_px = (image_width)/(2*np.tan(np.deg2rad(horizontal_fov)/2))
        
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
            # self.get_logger().info(f'Type = {type(depth_frame)}, Unique values {len(np.unique(depth_frame))}')
            real_depth_frame = self.disparity_to_depth(depth_frame)
            self.get_logger().info(f'Depth = {np.shape(real_depth_frame)}, Unique values {np.unique(real_depth_frame,return_counts=True)}')
        except CvBridgeError as e2:
            self.get_logger().error("Depth frame CV Bridge failed: "+str(e2))
        except Exception as e:
            self.get_logger().error("Depth frame conversion failed: "+str(e))
        if depth_frame is not None:
            depth_filename = "depth_"+str(curr_epoch_time)+".png"
            depth_filepath=PATH+"/depth"
            cv2.imwrite(os.path.join(depth_filepath,depth_filename),depth_frame)
            cv2.imshow('depth',depth_frame)
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

    def disparity_to_depth(self,disparity_frame):
        """ OAK D Pro Stereo pair 
            HFOV = 80 degrees 
            baseline = 0.075 m

            depth_m = fx_px * (baseline_m / disparity_px)
        """
        depth_frame = np.where(
            disparity_frame != 0,
            (self.focal_length_px * self.baseline) / disparity_frame,
            0
            )
        
        return depth_frame 

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
        


        