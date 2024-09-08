import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image,CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path, Odometry
from message_filters import TimeSynchronizer,Subscriber
import cv2
from cv_bridge import CvBridge,CvBridgeError
import numpy as np
from matplotlib import pyplot as plt
import tf2_ros
from tf2_ros import Buffer,TransformListener, TransformBroadcaster, LookupException, ConnectivityException, ExtrapolationException
from scipy.spatial.transform import Rotation as R

class VisualOdometryNode(Node):
    def __init__(self):
        super().__init__('visual_odom_node')

        self.K = np.array([[399.2978210449219, 0.0, 308.3040771484375], [0.0, 399.0950012207031, 197.78994750976562], [0.0, 0.0, 1.0]]) # Camera Calibration matrix 
                
        # Subscribe to depth camera - RGB and depth frames
        self.rgb_subscriber = Subscriber(self,Image,'/oak_pro/left_compressed')
        self.depth_subscriber = Subscriber(self,Image,'/oak_pro/depth_compressed')
        self.sync_subscriber = TimeSynchronizer([self.rgb_subscriber,self.depth_subscriber],10)
        self.sync_subscriber.registerCallback(self.frame_callback)
        self.vo_publisher = self.create_publisher(Odometry,'/visual_odom',10)
        # self.camera_info_subscriber = self.create_subscription(CameraInfo,'/depth_camera/camera_info',self.get_camera_info,10)
        self.path_publisher = self.create_publisher(Path,'/trajectory',10)
        self.odom_path_publisher = self.create_publisher(Path,'/trajectory',10)
        self.__visual_odom_tf_broadcaster = TransformBroadcaster(self)

        self.bridge=CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer,self)
        # Note: tf describes how to move from the source frame to the target frame.
        #       Ex - source base_link to target odom means we get pos and orientation of base link relative to odom ( Where is the base link in odom )

        self.publishTransform = False
        self.optical_to_base_transform = np.array([ # We take the base link source for this transform
            [0.000, 0.174, 0.985, 0.163],
            [-1.000, 0.000, 0.000, -0.037],
            [0.000, -0.985, 0.174, 0.045],
            [0.000, 0.000, 0.000, 1.000]
            ])
        # [ 0.000, -1.000,  0.000, -0.038],
        # [ 0.174,  0.000, -0.985,  0.016],
        # [ 0.985,  0.000,  0.174, -0.168],
        # [ 0.000,  0.000,  0.000,  1.000]
        
        self.curr_img_frame=None
        self.curr_depth_frame=None
        self.prev_img_frame=None
        self.prev_depth_frame=None
        self.kp_array=[]
        self.des_array=[]
        self.matches_array=[]
        self.curr_idx=0
        self.frame_count=0

        self.rot_total = np.eye(3)
        self.trans_total = np.zeros((3,1))
        self.translations_x = []
        self.translations_y = []
        self.translations_z = []
    
        """ Oak D Pro config """
        image_width = 640 # 400P setting for mono camera & stereo is scaled to mono
        horizontal_fov = 80 # deg
        self.baseline = 0.075 # m
        self.focal_length_px = self.K[0][0]#(image_width)/(2*np.tan(np.deg2rad(horizontal_fov)/2))
        
        
        self.robot_pose = np.zeros((1,4,4))

        self.robot_pose[0]= np.eye(4) #self.get_robot_tf() 

        self.camera_pose = np.zeros((1,4,4))
        self.camera_pose[0]=np.eye(4)
        
        self.trajectory = np.zeros((3, 1))

        # Initialize ORB detector
        self.orb = cv2.ORB_create()
        
        # Create SIFT and FLANN matcher
        self.sift = cv2.SIFT_create()
        
        FLANN_INDEX_KDTREE = 1
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=50)   # or pass empty dictionary
        self.flann = cv2.FlannBasedMatcher(index_params,search_params)

        # Path message
        self.path_msg = Path()

        # Visualization 
        cv2.namedWindow("depth",cv2.WINDOW_NORMAL)
        cv2.namedWindow("matches",cv2.WINDOW_NORMAL)

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

    def get_robot_tf(self):
        transformation = None
        try:
            transformation = self.tf_buffer.lookup_transform('odom','base_link',Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn(f'Transform between base link and odom not ready yet.')
        if transformation is not None:
            translation = [transformation.transform.translation.x,transformation.transform.translation.y,transformation.transform.translation.z]

    def frame_callback(self,rgb_msg,depth_msg):
        if self.frame_count %1 == 0:
            
            # Get grayscale and depth images
            try:
                img_frame = cv2.imdecode(np.frombuffer(rgb_msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                # img_frame = cv2.rotate(img_frame,cv2.ROTATE_180)
            except CvBridgeError as e1:
                self.get_logger().info(f'RGB frame CV Bridge failed : {e1}')
            try:
                disp_frame = cv2.imdecode(np.frombuffer(depth_msg.data, dtype=np.uint8), cv2.IMREAD_UNCHANGED)
                # disp_frame = cv2.rotate(disp_frame,cv2.ROTATE_180)
                depth_frame = disp_frame/1000 #self.disparity_to_depth(disp_frame)
                # self.get_logger().info(f'DEPTH at {np.shape(depth_frame)[0]//2} {np.shape(depth_frame)[1]//2} == {depth_frame[np.shape(depth_frame)[0]//2][np.shape(depth_frame)[1]//2]}')
            except CvBridgeError as e2:
                self.get_logger().info(f'Depth frame CV Bridge failed : {e2}')

            # Visualize
            # if img_frame is not None:
            #     cv2.namedWindow("grayscale",cv2.WINDOW_NORMAL)
            #     cv2.imshow("grayscale",img_frame)
            if depth_frame is not None:       
                cv2.namedWindow("depth",cv2.WINDOW_NORMAL)
                cv2.imshow("depth",depth_frame)
            
            # if cv2.waitKey(1)==ord('q'):
            #     raise SystemExit
            
            # img_frame = img_frame[200:,:]
            # depth_frame = depth_frame[200:,:]
            self.curr_img_frame=img_frame
            self.curr_depth_frame=depth_frame

            # Feature extraction
            kp,des = self.extract_frame_features(img_frame)
            # self.visualize_features(img_frame,kp)
            self.kp_array.append(kp)
            self.des_array.append(des)

            # If not first frame
            if self.prev_img_frame is not None:
                #Feature Matching
                match = self.match_feature(self.des_array[self.curr_idx-1],self.des_array[self.curr_idx])
                self.matches_array.append(match)
                self.visualize_matches(self.prev_img_frame,self.kp_array[self.curr_idx-1],self.curr_img_frame,self.kp_array[self.curr_idx],match)
                #Estimate Motion
                rmat, tvec, image1_points, image2_points = self.estimate_motion(match,self.kp_array[self.curr_idx-1],self.kp_array[self.curr_idx],self.K, self.prev_depth_frame)
                # self.visualize_camera_movement(self.prev_img_frame,image1_points,self.curr_img_frame,image2_points,True)
                
                this_transformation = np.eye(4)
                this_transformation[:3, :3] = rmat
                this_transformation[:3, 3] = np.squeeze(tvec)
                # self.get_logger().info(f'Translation: {tvec}')
                robot_transformation = self.optical_to_base_transform@this_transformation@np.linalg.inv(self.optical_to_base_transform)
                new_robot_pose = self.robot_pose[-1]@robot_transformation
                self.robot_pose=np.append(self.robot_pose,new_robot_pose.reshape(1,4,4),axis=0)
                self.trans_total += self.rot_total.dot(tvec)
                self.rot_total = rmat.dot(self.rot_total)
                self.get_logger().info(f'Translation: {self.trans_total}')
                self.get_logger().info(f'Rotation: {self.rot_total}')
                # self.get_logger().info(f'Translation = x:{self.robot_pose[-1][:3,3][0]}, y:{self.robot_pose[-1][:3,3][1]}, z:{self.robot_pose[-1][:3,3][2]} ')
                
                # Update Trajectory
                # self.update_trajectory(rmat,tvec)
                # self.publish_odometry(self.trans_total)
                #self.publish_path([float(i) for i in self.trans_total])

                self.publish_odometry(self.robot_pose[-1])
                self.publish_path([float(i) for i in self.robot_pose[-1][:3,3]])
            self.curr_idx +=1
            self.prev_depth_frame=self.curr_depth_frame
            self.prev_img_frame = self.curr_img_frame

        self.frame_count += 1

    def disparity_to_depth(self,disparity_frame):
        """ OAK D Pro Stereo pair 
            HFOV = 80 degrees 
            baseline = 0.075 m

            depth_m = fx_px * (baseline_m / disparity_px)
        """
        depth_frame = np.divide(
            self.focal_length_px * self.baseline,
            disparity_frame.astype(float),
            out=np.zeros_like(disparity_frame,dtype=float),
            where=(disparity_frame != 0)
        )
        return depth_frame 
      
    def extract_frame_features(self,image):
        #kp,des = self.orb.detectAndCompute(image,None)        
        kp,des = self.sift.detectAndCompute(image,None)
        
        return kp,des

    def match_feature(self,des1,des2):
        des1 = np.float32(des1)
        des2 = np.float32(des2)

        match_1 = self.flann.knnMatch(des1,des2,k=2)
        
        good_matches = []
        for m,n in match_1:
            if m.distance < 0.75*n.distance:
                good_matches.append(m)
        
        return good_matches
    
    def estimate_motion(self,match, kp1, kp2, k, depth1=None):
        """
        Estimate camera motion from a pair of subsequent image frames

        Arguments:
        match -- list of matched features from the pair of images
        kp1 -- list of the keypoints in the first image
        kp2 -- list of the keypoints in the second image
        k -- camera calibration matrix 
        
        Optional arguments:
        depth1 -- a depth map of the first frame. This argument is not needed if you use Essential Matrix Decomposition

        Returns:
        rmat -- recovered 3x3 rotation numpy matrix
        tvec -- recovered 3x1 translation numpy vector
        image1_points -- a list of selected match coordinates in the first image. image1_points[i] = [u, v], where u and v are 
                        coordinates of the i-th match in the image coordinate system
        image2_points -- a list of selected match coordinates in the second image. image1_points[i] = [u, v], where u and v are 
                        coordinates of the i-th match in the image coordinate system
                
        """
        rmat = np.eye(3)
        tvec = np.zeros((3, 1))
        image1_points = []
        image2_points = []
        
        objectpoints = []
        
        # Iterate through the matched features
        for m in match:
            # Get the pixel coordinates of features f[k - 1] and f[k]
            u1, v1 = kp1[m.queryIdx].pt
            u2, v2 = kp2[m.trainIdx].pt
            

            # Get the scale of features f[k - 1] from the depth map
            s = depth1[int(v1), int(u1)]
            
            # Check for valid scale values
            if s < 100.0 and s!=0:
                # Transform pixel coordinates to camera coordinates using the pinhole camera model
                p_c = np.linalg.inv(k) @ (s * np.array([u1, v1, 1]))
                
                # Save the results
                image1_points.append([u1, v1])
                image2_points.append([u2, v2])
                objectpoints.append(p_c)
            
        # Convert lists to numpy arrays
        objectpoints = np.vstack(objectpoints)
        imagepoints = np.array(image2_points)
        
        # Determine the camera pose from the Perspective-n-Point solution using the RANSAC scheme
        try:
            _, rvec, tvec, _ = cv2.solvePnPRansac(objectpoints, imagepoints, k, None)
            # Convert rotation vector to rotation matrix
            rmat, _ = cv2.Rodrigues(rvec)
            # self.get_logger().info(f'Translation : {tvec} Rotation : {rmat}')
        except:
            self.get_logger().warn(f'PNP failed due to less features ! ')
            rmat = np.eye(3)
            tvec = np.zeros((3, 1))
        
        return rmat, tvec, image1_points, image2_points
    
    def visualize_camera_movement(self, image1, image1_points, image2, image2_points, is_show_img_after_move=False):
        image1 = image1.copy()
        image2 = image2.copy()
        
        for i in range(0, len(image1_points)):
            # Coordinates of a point on t frame
            p1 = (int(image1_points[i][0]), int(image1_points[i][1]))
            # Coordinates of the same point on t+1 frame
            p2 = (int(image2_points[i][0]), int(image2_points[i][1]))

            cv2.circle(image1, p1, 5, (0, 255, 0), 1)
            cv2.arrowedLine(image1, p1, p2, (0, 255, 0), 1)
            cv2.circle(image1, p2, 5, (255, 0, 0), 1)

            if is_show_img_after_move:
                cv2.circle(image2, p2, 5, (255, 0, 0), 1)
        
        if is_show_img_after_move: 
            cv2.imshow("movement-2",image2)
            if cv2.waitKey(1)==ord('q'):
                cv2.destroyAllWindows()
                raise SystemExit
        else:
            cv2.imshow("movement-1",image1)
            if cv2.waitKey(1)==ord('q'):
                cv2.destroyAllWindows()
                raise SystemExit
            
    def update_trajectory(self,rmat,tvec):
        current_pose = np.eye(4)
        current_pose[0:3, 0:3] = rmat
        current_pose[0:3, 3] = tvec.T
        
        # Build the robot's pose from the initial position by multiplying previous and current poses
        camera_pose = self.camera_pose[-1] @ np.linalg.inv(current_pose)
        robot_pose = self.camera_optical_to_base_tf@camera_pose
        # self.get_logger().info(f'Pose:{robot_pose}')
        self.camera_pose=np.append(self.camera_pose,camera_pose.reshape(1,4,4),axis=0)
        self.robot_pose=np.append(self.robot_pose,robot_pose.reshape(1,4,4),axis=0)
        # Calculate current camera position from origin
        position = self.robot_pose[self.curr_idx] @ np.array([0., 0., 0., 1.])
        # self.get_logger().info(f'Position {self.curr_idx} : {position}')
        # Build trajectory
        self.trajectory=np.append(self.trajectory,position[0:3].reshape(3,1),axis=1)
        self.publish_path(position)
        # self.get_logger().info(f'Trajectory obtained {np.shape(self.trajectory)}')

    def publish_path(self,position):
        now_time = self.get_clock().now().to_msg()
        self.path_msg.header.frame_id='odom'
        self.path_msg.header.stamp = now_time
        this_pose = PoseStamped()
        this_pose.header.frame_id='odom'
        this_pose.header.stamp=now_time
        
        this_pose.pose.position.x = -position[0]
        this_pose.pose.position.y = position[1]     
        this_pose.pose.position.z = position[2]
        
        # try:
        #     # Get the transform from 'camera_optical_frame' to 'odom'
        #     transform = self.tf_buffer.lookup_transform('camera_optical_link', 'odom', rclpy.time.Time())
        #     # Transform the pose to the 'odom' frame
        #     transformed_point = self.tf_buffer.transform(this_pose, 'odom')
        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
        #     self.get_logger().error(f'Error: {e}')

        self.path_msg.poses.append(this_pose)
        self.path_publisher.publish(self.path_msg)

    def publish_odometry(self,transformation,position=None,orientation=None):
        vo_msg = Odometry()
        vo_msg.header.frame_id='odom'
        vo_msg.header.stamp=self.get_clock().now().to_msg()

        # this_transformation[:3, :3] = rmat
        # this_transformation[:3, 3] = tvec
        translation = transformation[:3, 3]
        vo_msg.pose.pose.position.x = float(translation[0])
        vo_msg.pose.pose.position.y = float(translation[1])
        vo_msg.pose.pose.position.z = float(translation[2])

        # rotation_mat = transformation[:3,:3] #np.dot(R_flip,transformation[:3,:3])
        rotation_obj = R.from_matrix(transformation[:3,:3])
        rotation_quaternion = rotation_obj.as_quat()
        
        vo_msg.pose.pose.orientation.x = rotation_quaternion[0]
        vo_msg.pose.pose.orientation.y = rotation_quaternion[1]
        vo_msg.pose.pose.orientation.z = rotation_quaternion[2]
        vo_msg.pose.pose.orientation.w = rotation_quaternion[3]

        self.vo_publisher.publish(vo_msg)

        if self.publishTransform:
            transform_ = TransformStamped()
            transform_.header.stamp=self.get_clock().now().to_msg()
            transform_.header.frame_id='odom'
            transform_._child_frame_id='base_link'

            transform_.transform.translation.x = vo_msg.pose.pose.position.x
            transform_.transform.translation.y = vo_msg.pose.pose.position.y
            transform_.transform.translation.z = vo_msg.pose.pose.position.z
            transform_.transform.rotation=vo_msg.pose.pose.orientation         
            
            self.__visual_odom_tf_broadcaster.sendTransform(transform_)


    def visualize_features(self,image,kp):
        """
        Visualize extracted features in the image

        Arguments:
        image -- a grayscale image
        kp -- list of the extracted keypoints

        Returns:
        """
        display = cv2.drawKeypoints(image, kp, None)
        cv2.imshow("features",display)
        if cv2.waitKey(1)==ord('q'):
            cv2.destroyAllWindows()
            raise SystemExit
        # plt.figure(figsize=(16,12),dpi=100)
        # plt.imshow(display)
        # plt.show()

    def visualize_matches(self,image1, kp1, image2, kp2, match):
        """
        Visualize corresponding matches in two images

        Arguments:
        image1 -- the first image in a matched image pair
        kp1 -- list of the keypoints in the first image
        image2 -- the second image in a matched image pair
        kp2 -- list of the keypoints in the second image
        match -- list of matched features from the pair of images

        Returns:
        image_matches -- an image showing the corresponding matches on both image1 and image2 or None if you don't use this function
        """
        image_matches = cv2.drawMatches(image1,kp1,image2,kp2,match,None,flags=2)
        cv2.imshow("matches",image_matches)
        if cv2.waitKey(1)==ord('q'):
            raise SystemExit
            
        # plt.figure(figsize=(16, 6), dpi=100)
        # plt.imshow(image_matches)
        # plt.show()


def main(args=None):
    rclpy.init(args=args)
    vo_slam_bot = VisualOdometryNode()
    try:
        rclpy.spin(vo_slam_bot)
    except (SystemExit,KeyboardInterrupt):
        rclpy.logging.get_logger("Quitting").info('Done')
        
    vo_slam_bot.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()