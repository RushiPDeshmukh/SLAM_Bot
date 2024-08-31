import depthai as dai
import numpy as np
import cv2



def click_event(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print('Mouse Clicked at:', x, y)

# OAK D PRO  Depth calibration -- focal length and HFOV can be taken from device as it changes with resolution
img_width_px = 640 
horizontal_fov = 43.715419733405426 # deg
focal_len_px = (img_width_px*0.5)/(np.tan(horizontal_fov*0.5*np.pi/180))
baseline = 0.075 # m

fps = 20  # Hz
# The disparity is computed at this resolution, then upscaled to RGB resolution. Oak D Pro mono camera run at 800P
monoResolution = dai.MonoCameraProperties.SensorResolution.THE_800_P

# Create pipeline
pipeline = dai.Pipeline()
device = dai.Device()
cameras = device.getConnectedCameras()
print(cameras)

# Define sources and outputs
left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
# Set manual exposure settings
exposure_time_us = 5000  # Example: 10000 microseconds (10ms)
sensitivity_iso = 800     # Example ISO value

left.initialControl.setManualExposure(exposure_time_us, sensitivity_iso)
right.initialControl.setManualExposure(exposure_time_us, sensitivity_iso)

stereo = pipeline.create(dai.node.StereoDepth)

imageOut = pipeline.create(dai.node.XLinkOut)
disparityOut = pipeline.create(dai.node.XLinkOut)

imageOut.setStreamName("image")
disparityOut.setStreamName("depth")

try:
    calibData = device.readCalibration2()
    print(f"LEFT - K: {calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B)}")
    print(f"Distortion coeff : {calibData.getDistortionCoefficients(dai.CameraBoardSocket.CAM_B)}")
except:
    raise

print("Se")
left.setResolution(monoResolution)
left.setCamera("left")
left.setFps(fps)

right.setResolution(monoResolution)
right.setCamera("right")
right.setFps(fps)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
# LR-check is required for depth alignment
stereo.setLeftRightCheck(True)
# stereo.setDepthAlign(dai.CameraBoardSocket.RGB)  # Align depth to RGB


## Filters
# stereo_config= stereo.initialConfig.get()
# stereo_config.postProcessing.speckleFilter.enable=True # Speckle Filter
# stereo_config.postProcessing.speckleFilter.speckleRange=50
# stereo_config.postProcessing.spatialFilter.enable=True # Spatial Filter
# stereo_config.postProcessing.spatialFilter.holeFillingRadius=2
# stereo_config.postProcessing.spatialFilter.numIterations=1
# stereo_config.postProcessing.thresholdFilter.minRange =  # Threshold Filter
# stereo_config.postProcessing.thresholdFilter.maxRange =  
# stereo.initialConfig.set(stereo_config)
# stereo.setExtendedDisparity(True)
# stereo.setSubpixel(True)

# Linking
left.out.link(imageOut.input)
left.out.link(stereo.left)
right.out.link(stereo.right)
stereo.disparity.link(disparityOut.input)

with device:
    print(device)
    device.startPipeline(pipeline)
    # Set IR projection
    device.setIrLaserDotProjectorBrightness(1.0) # in %, from 0 to 1 
    while True:
        latestPacket = {}
        latestPacket["image"] = None
        latestPacket["depth"] = None
        latestPacket['imu'] = None

        queueEvents = device.getQueueEvents(("image", "depth"))
        for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
                latestPacket[queueName] = packets[-1]

        if latestPacket["image"] is not None:
            frameImage = latestPacket["image"].getCvFrame()
            
        if latestPacket["depth"] is not None:
            frameDepth = latestPacket["depth"].getFrame()
            
        if frameDepth is not None and frameImage is not None:
            cv2.namedWindow('Depth')
            cv2.setMouseCallback('Depth', click_event)
            cv2.imshow('Depth', frameDepth)
            cv2.namedWindow('Image')
            cv2.imshow('Image', frameImage)
            
            cv2.waitKey(0)
            cv2.destroyAllWindows()