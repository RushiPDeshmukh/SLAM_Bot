import rclpy
import rclpy.logging
from rclpy.node import Node
from cv_bridge import CvBridge
import cv2
import numpy as np
import depthai as dai
from sensor_msgs.msg import Image , Imu, MagneticField
from rclpy.executors import MultiThreadedExecutor
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class OAK_Pro_Publisher(Node):
    def __init__(self):
        super().__init__('rgbd_publisher')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.image_pub = self.create_publisher(Image,'oak_pro/left_compressed',1)
        self.depth_pub = self.create_publisher(Image,'oak_pro/depth_compressed',1)
        self.imu_pub = self.create_publisher(Imu,'/imu/data_raw',qos_profile)
        # self.mag_pub = self.create_publisher(MagneticField,'/imu/mag',qos_profile)

        self.timestamp_rgb=None
        self.timestamp_depth=None
        
        #Save first depth frame
        self.first_depth = True

        self.bridge = CvBridge()
        
        # OAK D PRO  Depth calibration -- focal length and HFOV can be taken from device as it changes with resolution
        img_width_px = 640 
        horizontal_fov = 80 # deg 
        self.focal_len_px = (img_width_px*0.5)/(np.tan(horizontal_fov*0.5*np.pi/180))
        self.baseline = 0.075 # m
        
        fps = 60  # Hz
        # The disparity is computed at this resolution, then upscaled to RGB resolution. Oak D Pro mono camera run at 800P
        monoResolution = dai.MonoCameraProperties.SensorResolution.THE_400_P

        # Create pipeline
        self.pipeline = dai.Pipeline()
        self.device = dai.Device()

        # Define sources and outputs
        self.left = self.pipeline.create(dai.node.MonoCamera)
        self.right = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.IMU = self.pipeline.create(dai.node.IMU)

        # Set manual exposure settings
        # exposure_time_us = 5000  # Example: 10000 microseconds (10ms)
        # sensitivity_iso = 800     # Example ISO value

        # self.left.initialControl.setManualExposure(exposure_time_us, sensitivity_iso)
        # self.right.initialControl.setManualExposure(exposure_time_us, sensitivity_iso)

        self.imageOut = self.pipeline.create(dai.node.XLinkOut)
        self.ImuOut = self.pipeline.create(dai.node.XLinkOut)
        self.disparityOut = self.pipeline.create(dai.node.XLinkOut)

        self.imageOut.setStreamName("image")
        self.ImuOut.setStreamName("imu")
        self.disparityOut.setStreamName("depth")

        try:
            calibData = self.device.readCalibration2()
            self.get_logger().info(f"LEFT - K: {calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT,640,400)}")
            self.get_logger().info(f"Distortion coeff : {calibData.getDistortionCoefficients(dai.CameraBoardSocket.RIGHT)}")

        except:
            raise
        self.left.setResolution(monoResolution)
        self.left.setCamera("left")
        self.left.setFps(fps)

        self.right.setResolution(monoResolution)
        self.right.setCamera("right")
        self.right.setFps(fps)

        # IMU 
        self.IMU.enableIMUSensor([dai.IMUSensor.LINEAR_ACCELERATION,dai.IMUSensor.ROTATION_VECTOR],400)
        # self.IMU.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED,100)
        # self.IMU.enableIMUSensor(dai.IMUSensor.MAGNETOMETER_CALIBRATED,100)

        self.IMU.setBatchReportThreshold(1)
        self.IMU.setMaxBatchReports(10)

        # Stereo settings
        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        # LR-check is required for depth alignment
        self.stereo.setSubpixel(True)
        self.stereo.setSubpixelFractionalBits(3)
        # self.stereo.setDepthAlign(dai.CameraBoardSocket.RGB)  # Align depth to RGB

        ## Filters
        # stereo_config= self.stereo.initialConfig.get()
        # stereo_config.postProcessing.speckleFilter.enable=True # Speckle Filter
        # stereo_config.postProcessing.speckleFilter.speckleRange=50
        # stereo_config.postProcessing.spatialFilter.enable=True # Spatial Filter
        # stereo_config.postProcessing.spatialFilter.holeFillingRadius=2
        # stereo_config.postProcessing.spatialFilter.numIterations=1
        # stereo_config.postProcessing.thresholdFilter.minRange =  # Threshold Filter
        # stereo_config.postProcessing.thresholdFilter.maxRange =  
        # self.stereo.initialConfig.set(stereo_config)
        # self.stereo.setExtendedDisparity(True)
        # self.stereo.setSubpixel(True)
        
        # Linking
        # self.stereo.setOutputSize(640,400)
        self.stereo.rectifiedRight.link(self.imageOut.input) ## Rectified right added
        self.left.out.link(self.stereo.left)
        self.right.out.link(self.stereo.right)
        self.stereo.depth.link(self.disparityOut.input)
        self.IMU.out.link(self.ImuOut.input)

        #parameter
        self.frame_id = 'camera_link'

        self.frameGrabber()

        

    def frameGrabber(self):
        # Connect to device and start pipeline
        with self.device:
            self.device.startPipeline(self.pipeline)
            # Set IR projection
            self.device.setIrLaserDotProjectorBrightness(1.0) # in %, from 0 to 1 
            frameImage = None
            frameDepth = None

            while True:
                latestPacket = {}
                latestPacket["image"] = None
                latestPacket["depth"] = None
                latestPacket['imu'] = None

                queueEvents = self.device.getQueueEvents(("image", "depth","imu"))
                for queueName in queueEvents:
                    packets = self.device.getOutputQueue(queueName).tryGetAll()
                    if len(packets) > 0:
                        latestPacket[queueName] = packets[-1]

                if latestPacket["image"] is not None:
                    frameImage = latestPacket["image"].getCvFrame()
                    self.timestamp_image = latestPacket["image"].getTimestampDevice()
                    
                if latestPacket["depth"] is not None:
                    frameDepth = latestPacket["depth"].getFrame()
                    self.timestamp_depth = latestPacket["depth"].getTimestampDevice()
                    # self.get_logger().info(f'Depth: {type(frameDepth)}, Unique {len(np.unique(frameDepth))}')
                    
                if latestPacket["imu"] is not None:
                    imu_packet = latestPacket["imu"]
                    # self.timestamp_imu = latestPacket["imu"].getTimestampDevice()
                    self.imu_data_publisher(imu_packet.packets[0])

                if frameDepth is not None and frameImage is not None:
                    self.publish_rgbd_image(frameImage,frameDepth)
                    frameImage = None
                    frameDepth = None       
    
    def imu_data_publisher(self,imu_packet):
        #TO_DO: Sync the Accelerometer with Gyroscope
        # print(imu_packet, dir(imu_packet))
        acc_values = imu_packet.acceleroMeter
        # gyro_values = imu_packet.gyroscope
        # mag_value = imu_packet.magneticField
        rotation_vector = imu_packet.rotationVector
        IMU_msg = Imu()
        IMU_msg.header.stamp = self.get_clock().now().to_msg()
        IMU_msg.header.frame_id="imu_link"

        rot_vec_original = R.from_quat([rotation_vector.i,rotation_vector.j,rotation_vector.k,rotation_vector.real])
        shift_vec = R.from_euler('xyz',[0.0,-1.5708,0.0])
        shift_vec_1 = R.from_euler('xyz',[0.0,0.0,-1.5708])
        final_vec = (rot_vec_original*shift_vec*shift_vec_1).as_quat()

        # rot_vec_original = R.from_quat([rotation_vector.i,rotation_vector.j,rotation_vector.k,rotation_vector.real])
        # shift_vec = R.from_euler('xyz',[3.14159,0.0,1.5708])

        # final_vec = (rot_vec_original*shift_vec).as_quat()

        IMU_msg.orientation.x = -final_vec[0]
        IMU_msg.orientation.y = -final_vec[1]
        IMU_msg.orientation.z = final_vec[2]
        IMU_msg.orientation.w = final_vec[3]
        IMU_msg.linear_acceleration.x = acc_values.x
        IMU_msg.linear_acceleration.y = acc_values.y
        IMU_msg.linear_acceleration.z = acc_values.z
        # IMU_msg.angular_velocity.x = gyro_values.x
        # IMU_msg.angular_velocity.y = gyro_values.y
        # IMU_msg.angular_velocity.z = gyro_values.z
        
        # Mag_msg = MagneticField()
        # Mag_msg.header.stamp = self.get_clock().now().to_msg()
        # Mag_msg.header.frame_id="imu_link"
        # Mag_msg.magnetic_field.x = mag_value.x
        # Mag_msg.magnetic_field.y = mag_value.y
        # Mag_msg.magnetic_field.z = mag_value.z
        
        self.imu_pub.publish(IMU_msg)
        # self.mag_pub.publish(Mag_msg)
       

    def publish_rgbd_image(self, rgb_image, depth_image):
        timestamp = self.get_clock().now().to_msg()
        
        # Encode image with JPEG compression
        _, image_buffer = cv2.imencode('.png', rgb_image)
        

        # Create and publish compressed image message
        compressed_left_msg = Image()
        compressed_left_msg.header.frame_id = self.frame_id
        compressed_left_msg.header.stamp = timestamp
        compressed_left_msg.encoding = 'png'
        compressed_left_msg.data = image_buffer.tobytes()

        # Convert depth image to ROS Image message
        # Encode image with JPEG compression
        _, buffer = cv2.imencode('.png', depth_image)
        

        # Create and publish compressed image message
        compressed_msg = Image()
        compressed_msg.header.frame_id = self.frame_id
        compressed_msg.header.stamp = timestamp
        compressed_msg.encoding = 'png'
        compressed_msg.data = buffer.tobytes()
        # self.get_logger().info(f'Timestamp: rgb    {self.timestamp_rgb},  {timestamp} ')
        # self.get_logger().info(f'Timestamp: depth  {self.timestamp_depth},  {timestamp} ')
        
        self.image_pub.publish(compressed_left_msg)
        self.depth_pub.publish(compressed_msg)
        # self.get_logger().info("Published RGBD image ")

def main(args=None):
    rclpy.init(args=args)
    oak_pro_publisher = OAK_Pro_Publisher()

    executor = MultiThreadedExecutor()
    executor.add_node(oak_pro_publisher)

    try:
        executor.spin()
    except KeyboardInterrupt:
        rclpy.logging.get_logger("Quitting").info('Done')
    finally:
        executor.shutdown()
        oak_pro_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
