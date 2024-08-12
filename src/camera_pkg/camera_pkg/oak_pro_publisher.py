import rclpy
import rclpy.logging
from rclpy.node import Node
from camera_msgs.msg import RGBD
from cv_bridge import CvBridge
import cv2
import numpy as np
import depthai as dai
from sensor_msgs.msg import Image , Imu, MagneticField

class OAK_Pro_Publisher(Node):
    def __init__(self):
        super().__init__('rgbd_publisher')
        # self.publisher = self.create_publisher(RGBD, 'rgbd_frame', 10)
        self.image_pub = self.create_publisher(Image,'oak_pro/left',1)
        self.depth_pub = self.create_publisher(Image,'oak_pro/depth',1)
        self.imu_pub = self.create_publisher(Imu,'/imu/data_raw',1)
        self.mag_pub = self.create_publisher(MagneticField,'/imu/mag',1)

        self.timestamp_rgb=None
        self.timestamp_depth=None

        self.bridge = CvBridge()
        
        # OAK D PRO  Depth calibration
        img_width_px = 400 
        horizontal_fov = 80 # deg
        self.focal_len_px = (img_width_px*0.5)/(np.tan(horizontal_fov*0.5*np.pi/180))
        self.baseline = 0.075 # m
        
        fps = 30  # Hz
        # The disparity is computed at this resolution, then upscaled to RGB resolution. Oak D Pro mono camera run at 800P
        monoResolution = dai.MonoCameraProperties.SensorResolution.THE_400_P

        # Create pipeline
        self.pipeline = dai.Pipeline()
        self.device = dai.Device()

        # Define sources and outputs
        self.camRgb = self.pipeline.create(dai.node.Camera)
        self.left = self.pipeline.create(dai.node.MonoCamera)
        self.right = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)
        self.IMU = self.pipeline.create(dai.node.IMU)

        self.imageOut = self.pipeline.create(dai.node.XLinkOut)
        self.ImuOut = self.pipeline.create(dai.node.XLinkOut)
        self.disparityOut = self.pipeline.create(dai.node.XLinkOut)

        self.imageOut.setStreamName("image")
        self.ImuOut.setStreamName("imu")
        self.disparityOut.setStreamName("depth")

        #Properties
        rgbCamSocket = dai.CameraBoardSocket.CAM_A

        self.camRgb.setBoardSocket(rgbCamSocket)
        self.camRgb.setSize(1920, 1080) # 1080,720
        self.camRgb.setFps(fps)

        try:
            calibData = self.device.readCalibration2()
            lensPosition = calibData.getLensPosition(rgbCamSocket)
            print("RGB - K: ",calibData.getCameraIntrinsics(rgbCamSocket))
            if lensPosition:
                self.camRgb.initialControl.setManualFocus(lensPosition)
        except:
            raise
        self.left.setResolution(monoResolution)
        self.left.setCamera("left")
        self.left.setFps(fps)

        self.right.setResolution(monoResolution)
        self.right.setCamera("right")
        self.right.setFps(fps)

        self.IMU.enableIMUSensor(dai.IMUSensor.ACCELEROMETER,500)
        self.IMU.enableIMUSensor(dai.IMUSensor.GYROSCOPE_CALIBRATED,100)
        self.IMU.enableIMUSensor(dai.IMUSensor.MAGNETOMETER_CALIBRATED,100)

        self.IMU.setBatchReportThreshold(1)
        self.IMU.setMaxBatchReports(10)

        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        # LR-check is required for depth alignment
        self.stereo.setLeftRightCheck(True)


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
        
        # Linking
        self.left.out.link(self.imageOut.input)
        self.left.out.link(self.stereo.left)
        self.right.out.link(self.stereo.right)
        self.stereo.depth.link(self.disparityOut.input)
        self.IMU.out.link(self.ImuOut.input)

        #parameter
        self.frame_id = 'camera_frame'

        self.frameGrabber()

    def frameGrabber(self):
        # Connect to device and start pipeline
        with self.device:
            self.device.startPipeline(self.pipeline)
            # Set IR projection
            self.device.setIrLaserDotProjectorBrightness(0.5) # in %, from 0 to 1 
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
        # print(imu_packet)
        acc_values = imu_packet.acceleroMeter
        gyro_values = imu_packet.gyroscope
        mag_value = imu_packet.magneticField
        IMU_msg = Imu()
        Mag_msg = MagneticField()
        IMU_msg.header.stamp = self.get_clock().now().to_msg()
        IMU_msg.header.frame_id="imu"
        IMU_msg.linear_acceleration.x = acc_values.x
        IMU_msg.linear_acceleration.y = acc_values.y
        IMU_msg.linear_acceleration.z = acc_values.z
        IMU_msg.angular_velocity.x = gyro_values.x
        IMU_msg.angular_velocity.y = gyro_values.y
        IMU_msg.angular_velocity.z = gyro_values.z
        Mag_msg.header.stamp = self.get_clock().now().to_msg()
        Mag_msg.header.frame_id="imu"
        Mag_msg.magnetic_field.x = mag_value.x
        Mag_msg.magnetic_field.y = mag_value.y
        Mag_msg.magnetic_field.z = mag_value.z
        
        self.imu_pub.publish(IMU_msg)
        self.mag_pub.publish(Mag_msg)
       

    def publish_rgbd_image(self, gray_image, depth_image):
        timestamp = self.get_clock().now().to_msg()
        
        # Convert RGB image to ROS Image message
        image_msg = self.bridge.cv2_to_imgmsg(gray_image, encoding="mono8")
        image_msg.header.stamp = timestamp

        # Convert depth image to ROS Image message
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="16UC1") #8UC1 64FC1
        depth_msg.header.stamp = timestamp
        self.get_logger().info(f'Timestamp: rgb    {self.timestamp_rgb},  {timestamp} ')
        self.get_logger().info(f'Timestamp: depth  {self.timestamp_depth},  {timestamp} ')
        
        self.image_pub.publish(image_msg)
        self.depth_pub.publish(depth_msg)
        self.get_logger().info("Published RGBD image ")

def main(args=None):
    rclpy.init(args=args)
    oak_pro_publisher = OAK_Pro_Publisher()
    try:
        rclpy.spin(oak_pro_publisher)
    except KeyboardInterrupt:
        rclpy.logging.get_logger("Quitting").info('Done')
    oak_pro_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
