import numpy as np
import scipy.ndimage as ndi
from skimage.feature import peak_local_max

def pipeline_maxpeaks(img, min_dist=10, thresh_abs=10, num_peaks=100):
    imsize = np.shape(img)

    #img_sm = ndi.filters.gaussian_filter(img,7)     # Gaussian filter the image, to remove noise and so on, to get a better center estimate
    #img_sm = img
    allmaxcoords = peak_local_max(img, min_distance=int(min_dist), threshold_abs=thresh_abs)#, num_peaks=num_peaks)
    allmaxcoords = np.flip(np.transpose(allmaxcoords),0)
    #print(allmaxcoords)
    #print(type(allmaxcoords))

    #outputx = np.random.rand()*imsize[1]*scalex + centerx*imsize[1]
    #outputy= np.random.rand()*imsize[0]*scaley + centery*imsize[0]

    #coords = np.array([[outputx],
    #                   [outputy]])
    #coords = np.array([[outputx, outputx+10, outputx+20],
    #                   [outputy, outputy+10, outputy+20]])
    return allmaxcoords