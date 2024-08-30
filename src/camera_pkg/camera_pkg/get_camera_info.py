import depthai as dai
import numpy as np
import sys
from pathlib import Path
import math
import cv2

np.set_printoptions(suppress=True)

#resize intrinsics on host doesn't seem to work well for RGB 12MP
def resizeIntrinsicsFW(intrinsics, width, height, destWidth, destHeight, keepAspect=True):
    scaleH = destHeight / height
    scaleW = destWidth / width
    if keepAspect:
        scaleW = max(scaleW, scaleH)
        scaleH = scaleW
    
    scaleMat = np.array([[scaleW, 0, 0], [0, scaleH, 0], [0, 0, 1]])
    scaledIntrinscs = scaleMat @ intrinsics

    if keepAspect:
        if (scaleW * height > destHeight):
            scaledIntrinscs[1][2] -=(height * scaleW - destHeight) / 2.0
        elif (scaleW * width > destWidth):
            scaledIntrinscs[0][2] -= (width * scaleW - destWidth) / 2.0

    return scaledIntrinscs

def getHFov(intrinsics, width):
    fx = intrinsics[0][0]
    fov = 2 * 180 / (math.pi) * math.atan(width * 0.5 / fx)
    return fov

def getVFov(intrinsics, height):
    fy = intrinsics[1][1]
    fov = 2 * 180 / (math.pi) * math.atan(height * 0.5 / fy)
    return fov

def getDFov(intrinsics, w, h):
    fx = intrinsics[0][0]
    fy = intrinsics[1][1]
    return np.degrees(2*np.arctan(np.sqrt(w*w+h*h)/(((fx + fy) ))))

# Connect Device
with dai.Device() as device:

    calibData = device.readCalibration()

    cameras = device.getConnectedCameras()
    print(cameras)
    alpha = 1

    for cam in cameras:
        M, width, height = calibData.getDefaultIntrinsics(cam)
        M = np.array(M)
        d = np.array(calibData.getDistortionCoefficients(cam))

        hFov = getHFov(M, width)
        vFov = getHFov(M, height)
        dFov = getDFov(M, width, height)

        print("FOV measurement from calib (e.g. after undistortion):")
        print(f"{cam}")
        print(f"Horizontal FOV: {hFov}")
        print(f"Vertical FOV: {vFov}")
        print(f"Diagonal FOV: {dFov}")
        print()
        print("=============")
        print()

    M_rgb, width, height = calibData.getDefaultIntrinsics(dai.CameraBoardSocket.CAM_A)
    print("RGB Camera Default intrinsics...")
    print(f"Intrinsic : {M_rgb}")
    print(f"Image width: {width}, height: {height}")
    print("=============")

    f_x = width * (1 / (2 * math.tan((hFov / 2) * (math.pi / 180))))
    print(f"Calculated focal length = {f_x} ")
    print(f"Focal length from intrinsic matric = {calibData.getCameraIntrinsics(dai.CameraBoardSocket.RIGHT)[0][0]}")