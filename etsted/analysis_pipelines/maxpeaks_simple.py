import numpy as np
import scipy.ndimage as ndi
from skimage.feature import peak_local_max

def maxpeaks_simple(img, bkg=None, binary_mask=None, testmode=False, min_dist=10, thresh_abs=10, num_peaks=100):
    if bkg is None:
        return np.array([])
    img_sub = np.subtract(img, bkg)
    img_sm = ndi.filters.gaussian_filter(img_sub,2)     # small Gaussian filter to avoid detecting noise
    coords = peak_local_max(img_sm, min_distance=int(min_dist), threshold_abs=thresh_abs)#, num_peaks=num_peaks)
    coords = np.flip(np.transpose(coords),0)

    # if there are detected peaks: just return the first in the list. This is for testing, make a smarter choice later if multiple are detected (?)
    if coords.size:
        coords = np.array([[coords[0,0]],[coords[1,0]]])
    return coords