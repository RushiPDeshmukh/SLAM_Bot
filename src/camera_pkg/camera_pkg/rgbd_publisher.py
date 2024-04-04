import rclpy
from rclpy.node import Node
from camera_msgs.msg import RGBD
from cv_bridge import CvBridge
import cv2
import numpy as np
import depthai as dai

class RGBDPublisher(Node):
    def __init__(self):
        super().__init__('rgbd_publisher')
        self.publisher = self.create_publisher(RGBD, 'rgbd_frame', 10)
        self.bridge = CvBridge()
        self.timer = self.create_timer(1.0, self.publish_rgbd_image)

        fps = 30
        # The disparity is computed at this resolution, then upscaled to RGB resolution
        monoResolution = dai.MonoCameraProperties.SensorResolution.THE_480_P

        # Create pipeline
        self.pipeline = dai.Pipeline()
        self.device = dai.Device()

        # Define sources and outputs
        self.camRgb = self.pipeline.create(dai.node.Camera)
        self.left = self.pipeline.create(dai.node.MonoCamera)
        self.right = self.pipeline.create(dai.node.MonoCamera)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        self.rgbOut = self.pipeline.create(dai.node.XLinkOut)
        self.disparityOut = self.pipeline.create(dai.node.XLinkOut)

        self.rgbOut.setStreamName("rgb")
        
        self.disparityOut.setStreamName("disp")

        #Properties
        rgbCamSocket = dai.CameraBoardSocket.CAM_A

        self.camRgb.setBoardSocket(rgbCamSocket)
        self.camRgb.setSize(1920, 1080) # 1080,720
        self.camRgb.setFps(fps)

        try:
            calibData = self.device.readCalibration2()
            lensPosition = calibData.getLensPosition(rgbCamSocket)
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
        self.stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # LR-check is required for depth alignment
        self.stereo.setLeftRightCheck(True)
        self.stereo.setDepthAlign(rgbCamSocket)
        
        
        # Linking
        self.camRgb.video.link(self.rgbOut.input)
        self.left.out.link(self.stereo.left)
        self.right.out.link(self.stereo.right)
        self.stereo.disparity.link(self.disparityOut.input)

        #parameter
        self.frame_id = 'camera_frame'

        self.frameGrabber()

    def frameGrabber(self):
        # Connect to device and start pipeline
        with self.device:
            self.device.startPipeline(self.pipeline)

            frameRgb = None
            frameDisp = None

            while True:
                latestPacket = {}
                latestPacket["rgb"] = None
                latestPacket["disp"] = None

                queueEvents = self.device.getQueueEvents(("rgb", "disp"))
                for queueName in queueEvents:
                    packets = self.device.getOutputQueue(queueName).tryGetAll()
                    if len(packets) > 0:
                        latestPacket[queueName] = packets[-1]

                if latestPacket["rgb"] is not None:
                    frameRgb = latestPacket["rgb"].getCvFrame()
                    
                if latestPacket["disp"] is not None:
                    frameDisp = latestPacket["disp"].getFrame()
                    maxDisparity = self.stereo.initialConfig.getMaxDisparity()
                    # Optional, extend range 0..95 -> 0..255, for a better visualisation
                    if 1: frameDisp = (frameDisp * 255. / maxDisparity).astype(np.uint8)
                    
                if frameDisp is not None and frameRgb is not None:
                    self.publish_rgbd_image(frameRgb,frameDisp)
                    frameDisp = None
                    frameRgb = None
                

    def publish_rgbd_image(self, rgb_image, depth_image):
        # Convert RGB image to ROS Image message
        rgb_msg = self.bridge.cv2_to_imgmsg(rgb_image, encoding="bgr8")

        # Convert depth image to ROS Image message
        depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding="8UC1")

        # Create a new RGBD image message
        rgbd_msg = RGBD()
        rgbd_msg.header.stamp = self.get_clock().now().to_msg()
        rgbd_msg.rgb = rgb_msg
        rgbd_msg.depth = depth_msg

        #rgb_camera info
        rgbd_msg.rgb_camera_info.header.stamp = rgbd_msg.header.stamp
        rgbd_msg.rgb_camera_info.header.frame_id = self.frame_id
        
        #depth_camera_info
        rgbd_msg.depth_camera_info = rgbd_msg.rgb_camera_info

        # Publish the RGBD image message
        self.publisher.publish(rgbd_msg)
        self.get_logger().info("Published RGBD image")

def main(args=None):
    rclpy.init(args=args)
    rgbd_publisher = RGBDPublisher()
    rclpy.spin(rgbd_publisher)
    rgbd_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
