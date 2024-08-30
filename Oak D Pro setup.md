## Oak D Pro
#### Camera overview : https://docs.luxonis.com/hardware/products/OAK-D%20Pro

Stereo Camera - OV9282   
RGB Camera - Fixed Focus IMX378 (PY052)


### Stereo Depth 
[1] https://docs.luxonis.com/hardware/platform/depth/depth-accuracy#Depth%20Accuracy-Stereo%20Depth%20Accuracy-800P%2C%2075mm%20baseline%20distance%20OAKs
  



#### Settings used 
  FPS: 30 Hz
  RGB Image
    1920 x 1080 
  Depth Image
    800P scaled to RGB size
  Both images sent as mono8: CV_8UC1, grayscale image 

14 Hz achieved.

FPS: 10Hz
RGB Image --> grayscale
1920 x 1080
Depth Image
800P
Both images sent as mono8

RGB Intrinsic
K = [[3107.295654296875, 0.0, 1942.1912841796875], [0.0, 3105.113525390625, 1056.89697265625], [0.0, 0.0, 1.0]]


![image](https://github.com/user-attachments/assets/8ddf2f80-6914-471c-9bc4-30e3c11e3df6)

