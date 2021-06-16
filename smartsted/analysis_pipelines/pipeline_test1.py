import numpy as np

def pipeline_test1(img, bkg=None, binary_mask=None, scalex=0.1, scaley=0.1, centerx=0.5, centery=0.5):
    imsize = np.shape(img)
    outputx= np.random.rand()*imsize[0]*scaley + centery*imsize[0]
    outputy = np.random.rand()*imsize[1]*scalex + centerx*imsize[1]

    coords = np.array([[outputy],
                       [outputx]])
    return coords