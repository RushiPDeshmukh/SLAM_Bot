import cv2
import numpy as np
from utils import *
import rclpy
import VisualOdometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped, Quaternion
from nav_msgs.msg import Path, Odometry
from message_filters import TimeSynchronizer,Subscriber
import cv2
from cv_bridge import CvBridge,CvBridgeError
import numpy as np
from matplotlib import pyplot as plt
from tf2_ros import Buffer,TransformListener, TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
from scipy.spatial.transform import Rotation as R

class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__('visual_odom_node')
        self.k = np.array([[399.2978210449219, 0.0, 308.3040771484375], [0.0, 399.0950012207031, 197.78994750976562], [0.0, 0.0, 1.0]]) # Camera Calibration matrix 

        # Subscribe to depth camera - RGB and depth frames
        self.rgb_subscriber = Subscriber(self,Image,'/oak_pro/left_compressed')
        self.depth_subscriber = Subscriber(self,Image,'/oak_pro/depth_compressed')
        self.sync_subscriber = TimeSynchronizer([self.rgb_subscriber,self.depth_subscriber],10)
        self.sync_subscriber.registerCallback(self.frame_callback)
        self.vo_publisher = self.create_publisher(Odometry,'/visual_odom',10)
        self.odom_path_publisher = self.create_publisher(Path,'/trajectory',10)
        self.__visual_odom_tf_broadcaster = TransformBroadcaster(self)

        self.bridge=CvBridge()
        self.publishTransform = False
        self.optical_to_base_transform = np.array([ # We take the base link source for this transform
            [0.000, 0.174, 0.985, 0.163],
            [-1.000, 0.000, 0.000, -0.037],
            [0.000, -0.985, 0.174, 0.045],
            [0.000, 0.000, 0.000, 1.000]
            ])

        self.Visual_Odom = VisualOdometry(self.k, self.optical_to_base_transform)

        # Path message
        self.path_msg = Path()

        if self.publishTransform:
            transform_ = TransformStamped()
            transform_.header.stamp=self.get_clock().now().to_msg()
            transform_.header.frame_id='odom'
            transform_._child_frame_id='base_link'

            transform_.transform.translation.x = 0.0
            transform_.transform.translation.y = 0.0
            transform_.transform.translation.z = 0.0
            transform_.transform.rotation.x = 0.0
            transform_.transform.rotation.y = 0.0
            transform_.transform.rotation.z = 0.0
            transform_.transform.rotation.w = 1.0        
            
            self.__visual_odom_tf_broadcaster.sendTransform(transform_)        

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

        

        current_robot_pose = self.Visual_Odom.update(img_frame, depth_frame)
            
            
        self.publish_odometry(current_robot_pose,timestamp)
        self.publish_path([float(i) for i in current_robot_pose[-1][:3,3]],timestamp)
        self.last_timestamp = timestamp

        

    def publish_path(self,position,ts):

        self.path_msg.header.frame_id='odom'
        self.path_msg.header.stamp = ts
        this_pose = PoseStamped()
        this_pose.header.frame_id='odom'
        this_pose.header.stamp=ts
        
        this_pose.pose.position.x = position[0]
        this_pose.pose.position.y = position[1]     
        this_pose.pose.position.z = position[2]

        self.path_msg.poses.append(this_pose)
        self.odom_path_publisher.publish(self.path_msg)

    def publish_odometry(self,transformation,ts):
        vo_msg = Odometry()
        vo_msg.header.frame_id='odom'
        vo_msg.header.stamp=ts
        translation = transformation[:3, 3]
        vo_msg.pose.pose.position.x = float(translation[0])
        vo_msg.pose.pose.position.y = float(translation[1])
        vo_msg.pose.pose.position.z = float(translation[2])

        rotation_obj = R.from_matrix(transformation[:3,:3])
        rotation_quaternion = rotation_obj.as_quat()
        
        vo_msg.pose.pose.orientation.x = rotation_quaternion[0]
        vo_msg.pose.pose.orientation.y = rotation_quaternion[1]
        vo_msg.pose.pose.orientation.z = rotation_quaternion[2]
        vo_msg.pose.pose.orientation.w = rotation_quaternion[3]
        
        self.vo_publisher.publish(vo_msg)

        if self.publishTransform:
            transform_ = TransformStamped()
            transform_.header.stamp=ts
            transform_.header.frame_id='odom'
            transform_._child_frame_id='base_link'

            transform_.transform.translation.x = vo_msg.pose.pose.position.x
            transform_.transform.translation.y = vo_msg.pose.pose.position.y
            transform_.transform.translation.z = vo_msg.pose.pose.position.z
            transform_.transform.rotation=vo_msg.pose.pose.orientation         
            
            self.__visual_odom_tf_broadcaster.sendTransform(transform_)