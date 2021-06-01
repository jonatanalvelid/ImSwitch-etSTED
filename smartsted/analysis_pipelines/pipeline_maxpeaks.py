import numpy as np
import scipy.ndimage as ndi
from skimage.feature import peak_local_max

def pipeline_maxpeaks(img, min_dist=10, thresh_abs=10, num_peaks=100):
    imsize = np.shape(img)

    #img_sm = ndi.filters.gaussian_filter(img,7)     # Gaussian filter the image, to remove noise and so on, to get a better center estimate
    img_sm = img
    coords = peak_local_max(img_sm, min_distance=int(min_dist), threshold_abs=thresh_abs)#, num_peaks=num_peaks)
    #print(coords)
    coords = np.flip(np.transpose(coords),0)
    #coords = np.transpose(coords)
    #print(coords)
    #print(type(coords))

    #outputx = np.random.rand()*imsize[1]*scalex + centerx*imsize[1]
    #outputy= np.random.rand()*imsize[0]*scaley + centery*imsize[0]

    #coords = np.array([[outputx],
    #                   [outputy]])
    #coords = np.array([[outputx, outputx+10, outputx+20],
    #                   [outputy, outputy+10, outputy+20]])

    # if there are detected peaks: just return the first in the list. This is for testing, make a smarter choice later if multiple are detected (?)
    if coords.size:
        coords = np.array([[coords[0,0]],[coords[1,0]]])
    #print(coords)
    return coords