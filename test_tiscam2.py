from pyicic import IC_ImagingControl
import numpy as np

def grab_image():
    cam.reset_frame_ready()  # reset frame ready flag
    cam.send_trigger()
    frame, width, height, depth = cam.get_image_data()
    frame = np.array(frame, dtype='float64')
    frame = np.reshape(frame,(width,height,depth))[:,:,0]
    return frame

ic_ic = IC_ImagingControl.IC_ImagingControl()
ic_ic.init_library()
cam_names = ic_ic.get_unique_device_names()
print(cam_names)
cam = ic_ic.get_device(cam_names[1])
# print(self.cam.list_property_names())

cam.open()

roi_filter = cam.create_frame_filter('ROI'.encode('utf-8'))
cam.add_frame_filter_to_device(roi_filter)
cam.frame_filter_set_parameter(roi_filter, 'Top'.encode('utf-8'), 500)
cam.frame_filter_set_parameter(roi_filter, 'Left'.encode('utf-8'), 500)
cam.frame_filter_set_parameter(roi_filter, 'Height'.encode('utf-8'), 1500)
cam.frame_filter_set_parameter(roi_filter, 'Width'.encode('utf-8'), 1200)

cam.colorenable = 0
cam.gain.auto = False
cam.exposure.auto = False
cam.enable_continuous_mode(True)  # image in continuous mode
cam.start_live(show_display=False)  # start imaging
# self.cam.enable_trigger(True)  # camera will wait for trigger
# self.cam.send_trigger()
if not cam.callback_registered:
    cam.register_frame_ready_callback()  # needed to wait for frame ready callback

for i in range(0,30):
    frame = grab_image()
    #print(frame)
    print(np.mean(frame))

cam.close()