import numpy as np
import cupy as cp
from cupyx.scipy import ndimage as ndi
import cv2
from scipy.spatial import cKDTree, distance

def bapta_calcium_spikes(img, bkg=None, binary_mask=None, testmode=False, exinfo=None, min_dist=20, thresh_abs=0.2, num_peaks=5, noise_level=200, smoothing_radius=2, ensure_spacing=0, border_limit=10):
    f_multiply = 1e3
    #CHANGE 220304: change these initial checks slightly, to divide them in two separate.
    if binary_mask is None or np.shape(binary_mask) != np.shape(img):
        print('Binary mask not provided/wrong size.')
        binary_mask = cp.ones(np.shape(img)).astype('float32')
    if bkg is None or np.shape(img) != np.shape(bkg):
        print('You have to provide a background image for this pipeline.')
        img_ana = cp.zeros(np.shape(img)).astype('float32')
    else:
        img = cp.array(img).astype('float32')
        bkg = cp.array(bkg).astype('float32')

        # subtract last img (noisier, but quicker)
        img_ana = cp.subtract(img,bkg)
        
        # divide by last image to get percentual change in img
        img_div = bkg
        # replace noise with a very high value to avoid detecting noise
        img_div[img_div < noise_level] = 100000
        img_ana = cp.true_divide(img_ana, img_div)
        img_ana = img_ana * cp.array(binary_mask)
        
        img_ana = ndi.filters.gaussian_filter(img_ana, smoothing_radius)  # Gaussian filter the image, to remove noise and so on, to get a better center estimate

    "Peak_local_max all-in-one as a combo of opencv and cupy"
    thresh_abs = thresh_abs * f_multiply
    size = int(2 * min_dist + 1)
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


    coordinates = cp.fliplr(coordinates)
    coordinates = coordinates.get()

    # remove everything on the border (takes ~2-3ms if there are a lot of detected coordinates, but usually this is not the case)
    imsize = cp.shape(img)[0]
    idxremove = []
    for idx, coordpair in enumerate(coordinates):
        if coordpair[0] < border_limit or coordpair[0] > imsize - border_limit or coordpair[1] < border_limit or coordpair[1] > imsize - border_limit:
            idxremove.append(idx)
    coordinates = np.delete(coordinates,idxremove,axis=0)

    # remove everyhting down to a certain length
    if len(coordinates) > num_peaks:
        coordinates = coordinates[:int(num_peaks),:]

    if testmode:
        img_ana = img_ana.get()
        return coordinates, exinfo, img_ana
    else:
        return coordinates, exinfo
