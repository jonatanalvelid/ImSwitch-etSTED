import numpy as np
import scipy.ndimage as ndi
import cv2
from scipy.spatial import cKDTree, distance

def pipeline_fake(img, bkg=None, binary_mask=None, testmode=False, min_dist=30, thresh_abs=0.2, num_peaks=10, noise_level=1, smoothing_radius=2, ensure_spacing=1):
    
    coordinates = np.array([[50,50]])
    if testmode:
        return coordinates, img
    else:
        return coordinates