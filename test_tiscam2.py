from pyicic import IC_ImagingControl
import numpy as np
import time
import matplotlib.pyplot as plt

def grab_image(num):
    #cam.reset_frame_ready()  # reset frame ready flag
    #print('gi2')
    #cam.send_trigger()
    cam.wait_til_frame_ready(100)  # wait for frame ready
    #print('gi3')
    frame, width, height, depth = cam.get_image_data()
    #print('gi4')
    frame = np.array(frame, dtype='float64')
    #print('gi5')
    frame = np.reshape(frame,(width,height,depth))[:,:,0]
    print(f'{num}: {np.shape(frame)}')
    return frame

ic_ic = IC_ImagingControl.IC_ImagingControl()
ic_ic.init_library()
cam_names = ic_ic.get_unique_device_names()
print(cam_names)
cam = ic_ic.get_device(cam_names[1])
# print(self.cam.list_property_names())

cam.open()

#roi_filter = cam.create_frame_filter('ROI'.encode('utf-8'))
#cam.add_frame_filter_to_device(roi_filter)
#cam.frame_filter_set_parameter(roi_filter, 'Top'.encode('utf-8'), 500)
#cam.frame_filter_set_parameter(roi_filter, 'Left'.encode('utf-8'), 500)
#cam.frame_filter_set_parameter(roi_filter, 'Height'.encode('utf-8'), 1500)
#cam.frame_filter_set_parameter(roi_filter, 'Width'.encode('utf-8'), 1200)

cam.colorenable = 0
cam.gain.auto = False
cam.exposure.auto = False
cam.enable_continuous_mode(True)  # image in continuous mode
#cam.start_live(show_display=False)  # start imaging
cam.enable_trigger(False)  # camera will wait for trigger
# self.cam.send_trigger()
#if not cam.callback_registered:
#    cam.register_frame_ready_callback()  # needed to wait for frame ready callback

#for i in range(0,30):
#    cam.start_live()  # start imaging
#    frame = grab_image()
#    cam.stop_live()  # stop imaging
#    #print(frame)
#    print(np.mean(frame))
#    print(np.shape(frame))
#    if i == 15:
#        roi_filter = cam.create_frame_filter('ROI'.encode('utf-8'))
#        cam.add_frame_filter_to_device(roi_filter)
#        cam.frame_filter_set_parameter(roi_filter, 'Top'.encode('utf-8'), 500)
#        cam.frame_filter_set_parameter(roi_filter, 'Left'.encode('utf-8'), 500)
#        cam.frame_filter_set_parameter(roi_filter, 'Height'.encode('utf-8'), 1500)
#        cam.frame_filter_set_parameter(roi_filter, 'Width'.encode('utf-8'), 1200)
cam.reset_properties()
cam.start_live()
frame = grab_image('1')
#cam.suspend_live()
cam.stop_live()
roi_filter = cam.create_frame_filter('ROI'.encode('utf-8'))
cam.add_frame_filter_to_device(roi_filter)
cam.frame_filter_set_parameter(roi_filter, 'Top'.encode('utf-8'), 800)
cam.frame_filter_set_parameter(roi_filter, 'Left'.encode('utf-8'), 800)
cam.frame_filter_set_parameter(roi_filter, 'Height'.encode('utf-8'), 100)
cam.frame_filter_set_parameter(roi_filter, 'Width'.encode('utf-8'), 100)
top = cam.frame_filter_get_parameter(roi_filter, 'Top'.encode('utf-8'))
left = cam.frame_filter_get_parameter(roi_filter, 'Left'.encode('utf-8'))
hei = cam.frame_filter_get_parameter(roi_filter, 'Height'.encode('utf-8'))
wid = cam.frame_filter_get_parameter(roi_filter, 'Width'.encode('utf-8'))
cam.start_live()
for i in range(0,3):
    frameold = frame
    frame = grab_image('2')
    print(frame)
frameold = frame
frame = grab_image('3')


cam.stop_live()
cam.reset_properties()
#roi_filter2 = cam.create_frame_filter('ROI'.encode('utf-8'))
#cam.add_frame_filter_to_device(roi_filter)
cam.frame_filter_set_parameter(roi_filter, 'Top'.encode('utf-8'), 600)
cam.frame_filter_set_parameter(roi_filter, 'Left'.encode('utf-8'), 600)
cam.frame_filter_set_parameter(roi_filter, 'Height'.encode('utf-8'), 50)
cam.frame_filter_set_parameter(roi_filter, 'Width'.encode('utf-8'), 50)
top = cam.frame_filter_get_parameter(roi_filter, 'Top'.encode('utf-8'))
left = cam.frame_filter_get_parameter(roi_filter, 'Left'.encode('utf-8'))
hei = cam.frame_filter_get_parameter(roi_filter, 'Height'.encode('utf-8'))
wid = cam.frame_filter_get_parameter(roi_filter, 'Width'.encode('utf-8'))
cam.start_live()
for i in range(0,3):
    frameold = frame
    frame = grab_image('2')
    print(frame)
frameold = frame
frame = grab_image('3')

#plt.figure()
#plt.imshow(frame)
#plt.show()

cam.close()
