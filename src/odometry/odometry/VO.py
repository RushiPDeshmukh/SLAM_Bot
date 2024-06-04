#!/usr/bin/env python
# coding: utf-8

# # Visual Odometry for Localization in Autonomous Driving
#
# - Extract  features from the photographs  taken with a camera setup on the vehicle.
# - Use the extracted features to find matches between the features in different photographs.
# - Use the found matches to estimate the camera motion between subsequent photographs. 
# - Use the estimated camera motion to build the vehicle trajectory.



import numpy as np
import cv2
from matplotlib import pyplot as plt
import os
from tqdm import tqdm
# from attrdict import AttrDict



def get_data(folder_path):
   
    dataset_handler = {}

    #loading images
    dataset_handler['images_rgb'] = []
    dataset_handler['images'] = []
    for filename in tqdm(sorted(os.listdir(folder_path+'/rgb'))):
        img = cv2.imread(os.path.join(folder_path+'/rgb',filename))
        
        if img is not None:
            img = cv2.rotate(img,cv2.ROTATE_180)
            img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
            dataset_handler['images_rgb'].append(img)
            dataset_handler['images'].append(img_gray)
    #loading depth_maps
    dataset_handler['depth_maps'] = []
    for filename in tqdm(sorted(os.listdir(folder_path+'/depth'))):
        depth_map = cv2.imread(os.path.join(folder_path+'/depth',filename))
        
        if img is not None:
            depth_map = cv2.rotate(depth_map,cv2.ROTATE_180)
            dataset_handler['depth_maps'].append(depth_map)
    

    #k parameter
    dataset_handler['k'] = [[2994.451171875, 0.0, 2016.2567138671875], [0.0, 2994.451171875, 1080.2032470703125], [0.0, 0.0, 1.0]]
    return dataset_handler


def extract_features(image):
    """
    Find keypoints and descriptors for the image

    Arguments:
    image -- a grayscale image

    Returns:
    kp -- list of the extracted keypoints (features) in an image
    des -- list of the keypoint descriptors in an image
    """
    ### START CODE HERE ### 
    sift = cv2.SIFT_create()
    kp,des = sift.detectAndCompute(image,None)
    
#     xkp, des = orb.detectAndCompute(image,None)
    ### END CODE HERE ###
    
    return kp, des

def visualize_features(image, kp):
    """
    Visualize extracted features in the image

    Arguments:
    image -- a grayscale image
    kp -- list of the extracted keypoints

    Returns:
    """
    display = cv2.drawKeypoints(image, kp, None)
    plt.figure(figsize=(16,12),dpi=100)
    plt.imshow(display)
    plt.show()

def extract_features_dataset(images, extract_features_function):
    """
    Find keypoints and descriptors for each image in the dataset

    Arguments:
    images -- a list of grayscale images
    extract_features_function -- a function which finds features (keypoints and descriptors) for an image

    Returns:
    kp_list -- a list of keypoints for each image in images
    des_list -- a list of descriptors for each image in images
    
    """
    kp_list = []
    des_list = []
    
    ### START CODE HERE ###
    for i in tqdm(range(len(images))):
        
        kp,des = extract_features_function(images[i])
        kp_list.append(kp)
        des_list.append(des)
    ### END CODE HERE ###
    
    return kp_list, des_list

def match_features(des1, des2):
    """
    Match features from two images

    Arguments:
    des1 -- list of the keypoint descriptors in the first image
    des2 -- list of the keypoint descriptors in the second image

    Returns:
    match -- list of matched features from two images. Each match[i] is k or less matches for the same query descriptor
    """
    ### START CODE HERE ###
    
    FLANN_INDEX_KDTREE = 0
    index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
    search_params = dict(checks=50)   # or pass empty dictionary

    flann = cv2.FlannBasedMatcher(index_params,search_params)

    des1 = np.float32(des1)
    des2 = np.float32(des2)

    match_1 = flann.knnMatch(des1,des2,k=2)
    
    good = []
    for m,n in match_1:
        if m.distance < 0.6*n.distance:
            good.append(m)

    
    ### END CODE HERE ###

    return good

# Optional
def filter_matches_distance(match, dist_threshold):
    """
    Filter matched features from two images by distance between the best matches

    Arguments:
    match -- list of matched features from two images
    dist_threshold -- maximum allowed relative distance between the best matches, (0.0, 1.0) 

    Returns:
    filtered_match -- list of good matches, satisfying the distance threshold
    """
    filtered_match = []
    
    ### START CODE HERE ###
    for i,m in enumerate(match):
        if m.distance < dist_threshold:
            filtered_match.append(m)
    ### END CODE HERE ###

    return filtered_match

def visualize_matches(image1, kp1, image2, kp2, match):
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
    plt.figure(figsize=(16, 6), dpi=100)
    plt.imshow(image_matches)
    plt.show()


def match_features_dataset(des_list, match_features):
    """
    Match features for each subsequent image pair in the dataset

    Arguments:
    des_list -- a list of descriptors for each image in the dataset
    match_features -- a function which maches features between a pair of images

    Returns:
    matches -- list of matches for each subsequent image pair in the dataset. 
               Each matches[i] is a list of matched features from images i and i + 1
               
    """
    matches = []
    prev_des = des_list[0]
    
    ### START CODE HERE ###
    for i in tqdm(range(1,len(des_list))):
        match = match_features(prev_des,des_list[i])
        matches.append(match)
        prev_des = des_list[i]

    
    ### END CODE HERE ###
    
    return matches

# Optional
def filter_matches_dataset(filter_matches_distance, matches, dist_threshold):
    """
    Filter matched features by distance for each subsequent image pair in the dataset

    Arguments:
    filter_matches_distance -- a function which filters matched features from two images by distance between the best matches
    matches -- list of matches for each subsequent image pair in the dataset. 
               Each matches[i] is a list of matched features from images i and i + 1
    dist_threshold -- maximum allowed relative distance between the best matches, (0.0, 1.0) 

    Returns:
    filtered_matches -- list of good matches for each subsequent image pair in the dataset. 
                        Each matches[i] is a list of good matches, satisfying the distance threshold
               
    """
    filtered_matches = []
    
    ### START CODE HERE ###
    for match in tqdm(matches):
        filtered_matches.append(filter_matches_distance(match,dist_threshold))

    
    ### END CODE HERE ###
    
    return filtered_matches

def estimate_motion(match, kp1, kp2, k, depth1=None):
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
    for m in tqdm(match):
        # Get the pixel coordinates of features f[k - 1] and f[k]
        u1, v1 = kp1[m.queryIdx].pt
        u2, v2 = kp2[m.trainIdx].pt
        
        # Get the scale of features f[k - 1] from the depth map
        s = depth1[int(v1), int(u1)]
        print(s)
        
        # Check for valid scale values
        if s < 1000:
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
    _, rvec, tvec, _ = cv2.solvePnPRansac(objectpoints, imagepoints, k, None)
    
    # Convert rotation vector to rotation matrix
    rmat, _ = cv2.Rodrigues(rvec)
    
    return rmat, tvec, image1_points, image2_points

def estimate_trajectory(estimate_motion, matches, kp_list, k, depth_maps=[]):
    """
    Estimate complete camera trajectory from subsequent image pairs

    Arguments:
    estimate_motion -- a function which estimates camera motion from a pair of subsequent image frames
    matches -- list of matches for each subsequent image pair in the dataset. 
               Each matches[i] is a list of matched features from images i and i + 1
    des_list -- a list of keypoints for each image in the dataset
    k -- camera calibration matrix 
    
    Optional arguments:
    depth_maps -- a list of depth maps for each frame. This argument is not needed if you use Essential Matrix Decomposition

    Returns:
    trajectory -- a 3xlen numpy array of the camera locations, where len is the lenght of the list of images and   
                  trajectory[:, i] is a 3x1 numpy vector, such as:
                  
                  trajectory[:, i][0] - is X coordinate of the i-th location
                  trajectory[:, i][1] - is Y coordinate of the i-th location
                  trajectory[:, i][2] - is Z coordinate of the i-th location
                  
                  * Consider that the origin of your trajectory cordinate system is located at the camera position 
                  when the first image (the one with index 0) was taken. The first camera location (index = 0) is geven 
                  at the initialization of this function

    """
    trajectory = np.zeros((3, 1))
    
    ### START CODE HERE ###
     # Create variables for computation
    trajectory = np.zeros((3, len(matches) + 1))
    robot_pose = np.zeros((len(matches) + 1, 4, 4))

    # Initialize camera pose
    robot_pose[0] = np.eye(4)

    # Iterate through the matched features
    for i in range(len(matches)):
        # Estimate camera motion between a pair of images
        rmat, tvec, image1_points, image2_points = estimate_motion(matches[i], kp_list[i], kp_list[i + 1], k, depth_maps[i])

#         # Save camera movement visualization
#         if save:
#             image = visualize_camera_movement(dataset_handler.images_rgb[i], image1_points, dataset_handler.images_rgb[i + 1], image2_points)
#             plt.imsave('{}/frame_{:02d}.jpg'.format(save, i), image)

        # Determine current pose from rotation and translation matrices
        current_pose = np.eye(4)
        current_pose[0:3, 0:3] = rmat
        current_pose[0:3, 3] = tvec.T

        # Build the robot's pose from the initial position by multiplying previous and current poses
        robot_pose[i + 1] = robot_pose[i] @ np.linalg.inv(current_pose)

        # Calculate current camera position from origin
        position = robot_pose[i + 1] @ np.array([0., 0., 0., 1.])

        # Build trajectory
        trajectory[:, i + 1] = position[0:3]


    ### END CODE HERE ###
    
    return trajectory

if __name__ == "__main__":
    
    # Part 1. Features Extraction
    CWD_PATH = os.path.abspath(os.getcwd())
    PATH=CWD_PATH+'/images'
    
    print("Loading images")
    dataset_handler = get_data(PATH)

    image = dataset_handler["images"][0]
    print('imgs_loaded',str(np.shape(image)))
    # cv2.namedWindow("grayscale",cv2.WINDOW_NORMAL)
    # cv2.imshow("grayscale",image)

    image_rgb = dataset_handler["images_rgb"][0]
    # cv2.namedWindow("rgb",cv2.WINDOW_NORMAL)
    # cv2.imshow("rgb",image_rgb)

    i = 0
    depth = dataset_handler["depth_maps"][i]
    print(np.unique(depth))
    # cv2.namedWindow("depth",cv2.WINDOW_NORMAL)
    # cv2.imshow("depth",depth)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    images = dataset_handler["images"]
    print("Extracting Features")
    kp_list, des_list = extract_features_dataset(images, extract_features)
    
    # visualize_features(images[0],kp_list[0])
    # # Part II. Feature Matching
    print("Matching features")
    matches = match_features_dataset(des_list, match_features)
    # visualize_matches(images[0],kp_list[0],images[1],kp_list[1],matches[0])

    print("Filtering features")
    is_main_filtered_m = False
    if is_main_filtered_m:
        dist_threshold = 0.75
        filtered_matches = filter_matches_dataset(filter_matches_distance, matches, dist_threshold)
        matches = filtered_matches
        
    # Part III. Trajectory Estimation
    print("Estimating trajectory")
    depth_maps = dataset_handler['depth_maps']
    k = dataset_handler['k']
    trajectory = estimate_trajectory(estimate_motion, matches, kp_list, k, depth_maps=depth_maps)

    print("Trajectory X:\n {0}".format(trajectory[0,:].reshape((1,-1))))
    print("Trajectory Y:\n {0}".format(trajectory[1,:].reshape((1,-1))))
    print("Trajectory Z:\n {0}".format(trajectory[2,:].reshape((1,-1))))


    # ### Visualize your Results