import cv2
import numpy as np
from matplotlib import pyplot as plt
from scipy.spatial.transform import Rotation as R


class VisualOdometry():
    def __init__(self, camera_intrinsics, optical_to_robot_tf, feature_extractor = 'SIFT', nfeatures = 100, good_matches_threshold = 0.6) -> None:
        self.camera_intrinsics = camera_intrinsics
        self.optical_to_robot_tf = optical_to_robot_tf
        self.feature_extractor_to_use = feature_extractor
        
        #Parameters
        self.nfeatures = nfeatures
        self.good_matches_threshold = good_matches_threshold

        self.ref_image = None
        self.ref_image_features = None
        self.ref_depth_image = None

        self.current_robot_pose = np.eye(4)

        #Initialize feature extractor and FLANN matcher:
        if self.feature_extractor_to_use == "ORB":
            self.orb = cv2.ORB_create(nfeatures = nfeatures)
            FLANN_INDEX_LSH = 6
            index_params = dict(algorithm=FLANN_INDEX_LSH,
                                table_number=6,  # 12 is a good default
                                key_size=12,     # 20 is a good default
                                multi_probe_level=1)  # 2 is a good default
            search_params = dict(checks=50)  # The higher, the more accurate

        elif self.feature_extractor_to_use == "SIFT":
            self.sift = cv2.SIFT_create(nfeatures= nfeatures)
            FLANN_INDEX_KDTREE = 1
            index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
            search_params = dict(checks=50)  # Higher checks give better precision

        self.flann = cv2.FlannBasedMatcher(index_params, search_params)

    def feature_extractor(self, image):
        if self.feature_extractor_to_use == 'ORB':
            kp,des = self.orb.detectAndCompute(image,None)
        elif self.feature_extractor_to_use == 'SIFT':
            kp,des = self.sift.detectAndCompute(image,None)
        return kp,des
    
    def match_features(self, des1, des2):
        des1 = np.float32(des1)
        des2 = np.float32(des2)

        match_1 = self.flann.knnMatch(des1,des2,k=2)
        
        good_matches = []
        for m,n in match_1:
            if m.distance < self.good_matches_threshold*n.distance:
                good_matches.append(m)
        
        return good_matches

    def estimate_motion(self,match, kp1, kp2, k, depth1=None):

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
        except:
            rmat = np.eye(3)
            tvec = np.zeros((3, 1))
        
        return rmat, tvec, image1_points, image2_points

    def update(self,current_image,current_depth):
        
        if self.ref_image is None:
            # Feature extraction
            self.ref_image_features = self.feature_extractor(current_image)
            self.ref_image = current_image
            self.ref_depth_image = current_depth
        
        else:
            # Feature extraction
            kp,des = self.feature_extractor(current_image)

            #match the features with ref image features
            matches = self.match_features(self.ref_image_features[1],des)

            #Estimate motion of camera from ref image
            rmat, tvec, _, _ = self.estimate_motion(matches,self.ref_image_features[0],kp,self.camera_intrinsics,self.ref_depth_image)

            current_camera_pose = np.eye(4)
            current_camera_pose[0:3, 0:3] = rmat
            current_camera_pose[0:3, 3] = tvec.T
            current_camera_pose = np.linalg.inv(current_camera_pose)

            current_robot_transformation = self.optical_to_robot_tf@current_camera_pose@np.linalg.inv(self.optical_to_robot_tf)

            #updated robot_pose
            self.current_robot_pose = self.current_robot_pose@current_robot_transformation

            #update reference point
            self.ref_image = current_image
            self.ref_image_features = kp,des
            self.ref_depth_image = current_depth

        return self.current_robot_pose
    
