import numpy as np
import cupy as cp
from cupyx.scipy import ndimage as ndi
import cv2
from scipy.spatial import cKDTree, distance

def pipeline_maxpeaks_opt(img, bkg=None, binary_mask=None, testmode=False, min_dist=30, thresh_abs=0.2, num_peaks=10, noise_level=1, smoothing_radius=2, ensure_spacing=1):
    f_multiply = 100
    if bkg is None or binary_mask is None:
        print('You have to provide a background image and a binary mask for this pipeline!')
        img_ana = cp.zeros(cp.shape(cp.array(img)))
    else:
        img = cp.array(img)
        bkg = cp.array(bkg)
        # subtract last img (noisier, but quicker)
        img_ana = cp.subtract(img,bkg)
        
        # divide by last image to get percentual change in img
        img_div = bkg
        # replace noise with a very high value to avoid detecting noise
        img_div[img_div < noise_level] = 100000
        img_ana = cp.divide(img_ana, img_div)
        img_ana = img_ana * cp.array(binary_mask)
        
        img_ana = ndi.filters.gaussian_filter(img_ana, smoothing_radius)  # Gaussian filter the image, to remove noise and so on, to get a better center estimate

    "Peak_local_max all-in-one as a combo of opencv and cupy"
    thresh_abs = thresh_abs * f_multiply
    size = 2 * min_dist + 1
    img_ana = (img_ana * f_multiply).astype('uint16')
    # get filter structuring element
    footprint = cv2.getStructuringElement(cv2.MORPH_RECT, ksize=[size,size])
    # maximum filter (dilation + equal)
    image_max = cv2.dilate(img_ana.get(), kernel=footprint)
    #return image, image_max
    mask = cp.equal(img_ana,cp.array(image_max))
    mask &= cp.greater(img_ana, thresh_abs)
    
    # get coordinates of peaks
    coordinates = cp.nonzero(mask)
    intensities = img_ana[coordinates]
    # highest peak first
    idx_maxsort = cp.argsort(-intensities).get()
    coordinates = tuple(arr.get() for arr in coordinates)
    coordinates = np.transpose(coordinates)[idx_maxsort]
    
    if ensure_spacing==1:
        output = coordinates
        if len(coordinates):
            coordinates = cp.asnumpy(coordinates)
            # Use KDtree to find the peaks that are too close to each other
            tree = cKDTree(coordinates, balanced_tree=False, compact_nodes=False, leafsize=50)

            indices = tree.query_ball_point(coordinates, workers=1, r=min_dist, p=cp.inf, return_sorted=False)
            rejected_peaks_indices = set()
            for idx, candidates in enumerate(indices):
                if idx not in rejected_peaks_indices:
                    # keep current point and the points at exactly spacing from it
                    candidates.remove(idx)
                    dist = distance.cdist(
                        [coordinates[idx]],
                        coordinates[candidates],
                        distance.minkowski,
                        p=cp.inf,
                    ).reshape(-1)
                    candidates = [
                        c for c, d in zip(candidates, dist) if d < min_dist
                    ]

                    # candidates.remove(keep)
                    rejected_peaks_indices.update(candidates)

            # Remove the peaks that are too close to each other
            output = np.delete(coordinates, tuple(rejected_peaks_indices), axis=0)

        coordinates = cp.array(output)
    else:
        coordinates = cp.array(coordinates)

    if len(coordinates) > num_peaks:
        coordinates = coordinates[:num_peaks]

    if testmode:
        return coordinates, img_ana
    else:
        return coordinates
