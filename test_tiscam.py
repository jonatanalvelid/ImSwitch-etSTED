from pyicic import IC_ImagingControl
import numpy as np
import matplotlib.pyplot as plt


def grabFrame():
    cam.reset_frame_ready()  # reset frame ready flag
    cam.send_trigger()
    # self.cam.wait_til_frame_ready(0)  # wait for frame ready due to trigger
    data = cam.get_image_data()
    # Prev: averaging the RGB image to a grayscale. Very slow for the big camera (2480x2048).
    #frame = np.average(frame, 2)
    # New: just take the R-component, this should anyway contain most information in both cameras. Change this if we want to look at another color, like GFP!
    frame = np.array(data[0], dtype='float64')
    print(np.mean(frame))
    # Check if below is giving the right dimensions out
    #frame = np.ndarray(buffer=data, dtype='uint8', shape=(height, width, depth))[:,:,0]
    frame = np.reshape(frame,(data[1],data[2],3))[:,:,0]
    return frame


ic_ic = IC_ImagingControl.IC_ImagingControl()
ic_ic.init_library()
cam_names = ic_ic.get_unique_device_names()
#print(cam_names)
model = cam_names[0]
cam = ic_ic.get_device(model)
cam.open()
#print(cam.get_image_description())

#cam.show_property_dialog()

#roi_filter = cam.create_frame_filter('ROI'.encode('utf-8'))
#cam.add_frame_filter_to_device(roi_filter)
#cam.frame_filter_set_parameter(roi_filter, 'Top'.encode('utf-8'), 0)
#cam.frame_filter_set_parameter(roi_filter, 'Left'.encode('utf-8'), 0)
#cam.frame_filter_set_parameter(roi_filter, 'Height'.encode('utf-8'), 2048)
#cam.frame_filter_set_parameter(roi_filter, 'Width'.encode('utf-8'), 2448)

cam.colorenable = 0
cam.gain.auto = False
cam.exposure.auto = False

cam.enable_continuous_mode(True)
cam.start_live(show_display=False)
if not cam.callback_registered:
    cam.register_frame_ready_callback()
#print(cam.list_property_names())
#print(cam.gain.value)

img = grabFrame()
print(np.shape(img))
print(np.mean(img))

#plt.figure()
#plt.imshow(img)
#plt.show()

