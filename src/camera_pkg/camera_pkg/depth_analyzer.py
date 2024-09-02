import cv2
import numpy as np
import matplotlib.pyplot as plt

# Load the depth image using OpenCV
disparity_frame = cv2.imread('/home/jidnyesha/SLAM_Bot/images/depth/depth_1723921568.451576.png', cv2.IMREAD_UNCHANGED)

image_width = 640 # 400P setting for mono camera & stereo is scaled to mono
horizontal_fov = 80 # deg
baseline = 0.075 # m
focal_length_px = (image_width)/(2*np.tan(np.deg2rad(horizontal_fov)/2))
print(np.shape(disparity_frame))
depth_image = np.where(
            disparity_frame != 0,
            (focal_length_px * baseline) / disparity_frame,
            0
            )
        
# Normalize the depth image for visualization (optional)
normalized_depth = cv2.normalize(disparity_frame, None, 0, 255, cv2.NORM_MINMAX)
normalized_depth = np.uint8(normalized_depth)

# Create a figure and a set of subplots
fig, ax = plt.subplots()

# Display the depth image
im = ax.imshow(normalized_depth, cmap='gray')

# Function to display the depth value on click
def onclick(event):
    if event.xdata is not None and event.ydata is not None:
        x = int(event.xdata)
        y = int(event.ydata)
        if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
            z = depth_image[y, x]
            print(f'Clicked at (x={x}, y={y}) with depth value={z}')

# Connect the onclick function to the figure
fig.canvas.mpl_connect('button_press_event', onclick)

plt.show()

while True:
    cv2.namedWindow('Depth')
    cv2.imshow('Depth', disparity_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        cv2.destroyAllWindows()
        break
# def disparity_to_depth(self,disparity_frame):
#         """ OAK D Pro Stereo pair 
#             HFOV = 80 degrees 
#             baseline = 0.075 m

#             depth_m = fx_px * (baseline_m / disparity_px)
#         """
#         depth_frame = np.where(
#             disparity_frame != 0,
#             (self.focal_length_px * self.baseline) / disparity_frame,
#             0
#             )
        
#         return depth_frame