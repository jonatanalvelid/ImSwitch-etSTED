import numpy as np

def pipeline_test1(img, img_prev=None, scalex=0.1, scaley=0.1, centerx=0.5, centery=0.5):
    imsize = np.shape(img)
    outputx= np.random.rand()*imsize[0]*scaley + centery*imsize[0]
    outputy = np.random.rand()*imsize[1]*scalex + centerx*imsize[1]

    coords = np.array([[outputy],
                       [outputx]])
    #coords = np.array([[outputx, outputx+10, outputx+20],
    #                   [outputy, outputy+10, outputy+20]])
    return coords