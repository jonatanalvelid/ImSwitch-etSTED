import numpy as np
import scipy.ndimage as ndi
from skimage.feature import peak_local_max

def pipeline_maxpeaks(img, bkg=None, binary_mask=None, testmode=False, min_dist=10, thresh_abs=10, num_peaks=100):
    # USE 2px SMOOTHING NORMALLY
    #img_sm = ndi.filters.gaussian_filter(img,2)     # Gaussian filter the image, to remove noise and so on, to get a better center estimate
    img_sm = img
    coords = peak_local_max(img_sm, min_distance=int(min_dist), threshold_abs=thresh_abs)#, num_peaks=num_peaks)
    coords = np.flip(coords,axis=1)

    ## if there are detected peaks: just return the first in the list. This is for testing, make a smarter choice later if multiple are detected (?)
    #if coords.size:
    #    coords = np.array([[coords[0,0]],[coords[1,0]]])
    
    if testmode:
        return coords, img_sm
    else:
        return coords