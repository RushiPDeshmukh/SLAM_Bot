from VisualOdometry import *

from matplotlib import pyplot as plt
import os
from tqdm import tqdm
import numpy as np
import cv2

def import_images(path):
    dataset_handler = {}
    #loading images
    dataset_handler['images_gray'] = []
    dataset_handler['images'] = []
    for filename in tqdm(sorted(os.listdir(path+'/gray'))):
        img = cv2.imread(os.path.join(path+'/gray',filename))
        
        if img is not None:
            dataset_handler['images_gray'].append(img)
    #loading depth_maps
    dataset_handler['depth_maps'] = []
    for filename in tqdm(sorted(os.listdir(path+'/depth'))):
        depth_map = cv2.imread(os.path.join(path+'/depth',filename))
        
        if img is not None:
            dataset_handler['depth_maps'].append(depth_map)
    return dataset_handler

if __name__ == "__main__":
    
    CWD_PATH = os.path.abspath(os.getcwd())
    PATH=CWD_PATH+'/images'
    dataset_handler = import_images(PATH)
    
    k = np.array([[399.2978210449219, 0.0, 308.3040771484375], [0.0, 399.0950012207031, 197.78994750976562], [0.0, 0.0, 1.0]]) # Camera Calibration matrix 
                
    optical_to_base_tf = np.array([ # We take the base link source for this transform
            [0.000, 0.174, 0.985, 0.163],
            [-1.000, 0.000, 0.000, -0.037],
            [0.000, -0.985, 0.174, 0.045],
            [0.000, 0.000, 0.000, 1.000]
            ])
    vo = VisualOdometry(k,optical_to_base_tf)

    