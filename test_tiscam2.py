from pyicic import IC_ImagingControl
import numpy as np

def grab_image():
    cam.reset_frame_ready()  # reset frame ready flag
    cam.send_trigger()
    frame = cam.get_image_data()
    frame = np.array(frame[0], dtype='float64')
    frame = np.reshape(frame,(1024,1280,3))[:,:,0]
    return frame

ic_ic = IC_ImagingControl.IC_ImagingControl()
ic_ic.init_library()
cam_names = ic_ic.get_unique_device_names()
print(cam_names)
cam = ic_ic.get_device(cam_names[0])
# print(self.cam.list_property_names())

cam.open()

cam.colorenable = 0
cam.gain.auto = False
cam.exposure.auto = False
cam.enable_continuous_mode(True)  # image in continuous mode
cam.start_live(show_display=False)  # start imaging
# self.cam.enable_trigger(True)  # camera will wait for trigger
# self.cam.send_trigger()
if not cam.callback_registered:
    cam.register_frame_ready_callback()  # needed to wait for frame ready callback

frame = grab_image()
print(frame)
print(np.mean(frame))