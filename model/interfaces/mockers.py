import ctypes
import ctypes.util
import logging

import numpy as np
from lantz import Action, Feat
from lantz import Driver
from lantz import Q_

from scipy.stats import multivariate_normal

logging.basicConfig(level=logging.DEBUG, format='%(asctime)s %(message)s',
                    datefmt='%Y-%d-%m %H:%M:%S')


class constants:

    def __init__(self):
        self.GND = 0


class MockLaser(Driver):

    def __init__(self):
        super().__init__()

        self.mW = Q_(1, 'mW')

        self.enabled = False
        self.power_sp = 0 * self.mW
        self._digMod = False

    @property
    def idn(self):
        return 'Simulated laser'

    @property
    def status(self):
        """Current device status
        """
        return 'Simulated laser status'

    # ENABLE LASER
    @property
    def enabled(self):
        """Method for turning on the laser
        """
        return self.enabled_state

    @enabled.setter
    def enabled(self, value):
        self.enabled_state = value

    # LASER'S CONTROL MODE AND SET POINT

    @property
    def power_sp(self):
        """To handle output power set point (mW) in APC Mode
        """
        return self.power_setpoint

    @power_sp.setter
    def power_sp(self, value):
        self.power_setpoint = value

    # LASER'S CURRENT STATUS

    @property
    def power(self):
        """To get the laser emission power (mW)
        """
        return 55555 * self.mW

    def enter_mod_mode(self):
        self._digMod = True

    @property
    def digital_mod(self):
        """digital modulation enable state
        """
        return self._digMod

    @digital_mod.setter
    def digital_mod(self, value):
        self._digMod = value

    def mod_mode(self):
        """Returns the current operating mode
        """
        return 0

    @Feat(units='mW')
    def power_mod(self):
        return 0

    @power_mod.setter
    def power_mod(self, value):
        pass

    def query(self, text):
        return 0


class HMockCamData:

    # __init__
    #
    # Create a data object of the appropriate size.
    #
    # @param size The size of the data object in bytes.
    #
    def __init__(self, size, max_value):
        self.np_array = np.random.randint(1, max_value, int(size))
        self.size = size

    # __getitem__
    #
    # @param slice The slice of the item to get.
    #
    def __getitem__(self, slice):
        return self.np_array[slice]

    # copyData
    #
    # Uses the C memmove function to copy data from an address in memory
    # into memory allocated for the numpy array of this object.
    #
    # @param address The memory address of the data to copy.
    #
    def copyData(self, address):
        ctypes.memmove(self.np_array.ctypes.data, address, self.size)

    # getData
    #
    # @return A numpy array that contains the camera data.
    #
    def getData(self):
        return self.np_array

    # getDataPtr
    #
    # @return The physical address in memory of the data.
    #
    def getDataPtr(self):
        return self.np_array.ctypes.data


class MockHamamatsu(Driver):

    def __init__(self):

        self.buffer_index = 0
        self.camera_id = 9999
        self.camera_model = b'Mock Hamamatsu camera'
        self.debug = False
        self.frame_x = 500
        self.frame_y = 500
        self.frame_bytes = self.frame_x * self.frame_y * 2
        self.last_frame_number = 0
        self.properties = {}
        self.max_backlog = 0
        self.number_image_buffers = 0
        self.hcam_data = []
        self.flipimage = (False,False)

        self.mock_data_max_value = np.int(20000)
        self.mock_acquisiton_running = False

        self.s = Q_(1, 's')

        # Open the camera.
#        self.camera_handle = ctypes.c_void_p(0)
#        self.checkStatus(dcam.dcam_open(ctypes.byref(self.camera_handle),
#                                        ctypes.c_int32(self.camera_id),
#                                        None),
#                         "dcam_open")
        # Get camera properties.
        self.properties = {'Name': 'MOCK Hamamatsu',
                           'exposure_time': 9999,  # * self.s,
                           'accumulation_time': 99999,  # * self.s,
                           'image_width': 800,
                           'image_height': 800,
                           'image_framebytes': 8,
                           'subarray_hsize': 800,
                           'subarray_vsize': 800,
                           'subarray_mode': 'OFF',
                           'timing_readout_time': 9999,
                           'internal_frame_rate': 9999,
                           'internal_frame_interval': 9999,
                           'trigger_source': 1,
                           'trigger_mode': 1}

        # Get camera max width, height.
        self.max_width = self.getPropertyValue("image_width")[0]
        self.max_height = self.getPropertyValue("image_height")[0]

    def captureSetup(self):
        ''' (internal use only). This is called at the start of new acquisition
        sequence to determine the current ROI and get the camera configured
        properly.'''
        self.buffer_index = -1
        self.last_frame_number = 0

        # Set sub array mode.
        self.setSubArrayMode()

        # Get frame properties.
        self.frame_x = int(self.getPropertyValue("image_width")[0])
        self.frame_y = int(self.getPropertyValue("image_height")[0])
        self.frame_bytes = self.getPropertyValue("image_framebytes")[0]

    def checkStatus(self, fn_return, fn_name="unknown"):
        ''' Check return value of the dcam function call. Throw an error if
        not as expected?
        @return The return value of the function.'''
        pass

    def getFrames(self):
        ''' Gets all of the available frames.

        This will block waiting for new frames even if there new frames
        available when it is called.

        @return (frames, (frame x size, frame y size))'''
        frames = []
        frame_x, frame_y = self.frame_x, self.frame_y

        for i in range(2):
            # Create storage
            hc_data = HMockCamData(frame_x * frame_y, self.mock_data_max_value)
            frames.append(np.reshape(hc_data.getData(), (frame_x, frame_y)))

        return frames, (frame_x, frame_y)

    def getLast(self):
        frame_x, frame_y = self.frame_x, self.frame_y
        hc_data = HMockCamData(frame_x * frame_y, self.mock_data_max_value)
        return np.reshape(hc_data.getData(), (frame_x, frame_y))

    def getModelInfo(self):
        ''' Returns the model of the camera

        @param camera_id The (integer) camera id number.

        @return A string containing the camera name.'''
        return ('WARNING!: This is a Mock Version of the Hamamatsu Orca flash '
                'camera')

    def getProperties(self):
        ''' Return the list of camera properties. This is the one to call if you
        want to know the camera properties.

        @return A dictionary of camera properties.'''
        return self.properties

    def getPropertyAttribute(self, property_name):
        ''' Return the attribute structure of a particular property.

        FIXME (OPTIMIZATION): Keep track of known attributes?

        @param property_name The name of the property to get the attributes of.

        @return A DCAM_PARAM_PROPERTYATTR object.'''
        pass

    # getPropertyText
    #
    # Return the text options of a property (if any).
    #
    # @param property_name The name of the property to get the text values of.
    #
    # @return A dictionary of text properties (which may be empty).
    #
    def getPropertyText(self, property_name):
        pass

    # getPropertyRange
    #
    # Return the range for an attribute.
    #
    # @param property_name The name of the property (as a string).
    #
    # @return (minimum value, maximum value)
    #
    def getPropertyRange(self, property_name):
        pass

    # getPropertyRW
    #
    # Return if a property is readable / writeable.
    #
    # @return (True/False (readable), True/False (writeable))
    #
    def getPropertyRW(self, property_name):
        pass

    # getPropertyVale
    #
    # Return the current setting of a particular property.
    #
    # @param property_name The name of the property.
    #
    # @return (the property value, the property type)
    #
    def getPropertyValue(self, property_name):

        prop_value = self.properties[property_name]
        prop_type = property_name

        return prop_value, prop_type

    # isCameraProperty
    #
    # Check if a property name is supported by the camera.
    #
    # @param property_name The name of the property.
    #
    # @return True/False if property_name is a supported camera property.
    #
    def isCameraProperty(self, property_name):
        if (property_name in self.properties):
            return True
        else:
            return False

    # newFrames
    #
    # Return a list of the ids of all the new frames since the last check.
    #
    # This will block waiting for at least one new frame.
    #
    # @return [id of the first frame, .. , id of the last frame]
    #
    def newFrames(self):

        # Create a list of the new frames.
        new_frames = [0]

        return new_frames

    # setPropertyValue
    #
    # Set the value of a property.
    #
    # @param property_name The name of the property.
    # @param property_value The value to set the property to.
    #
    def setPropertyValue(self, property_name, property_value):

        # Check if the property exists.
        if not (property_name in self.properties):
            return False

        # Some values are not changeable while the acquisition is running
        if (self.mock_acquisiton_running and
            (property_name == 'subarray_vpos' or property_name == 'subarray_hpos' or
             property_name == 'subarray_vsize' or property_name == 'subarray_hsize')):
            raise Exception('Value not changeable while acquisition is running')

        # If the value is text, figure out what the
        # corresponding numerical property value is.

        self.properties[property_name] = property_value
#        print(property_name, 'set to:', self.properties[property_name])
#            if (property_value in text_values):
#                property_value = float(text_values[property_value])
#            else:
#                print(" unknown property text value:", property_value, "for",
#                      property_name)
#                return False
        return property_value

    # setSubArrayMode
    #
    # This sets the sub-array mode as appropriate based on the current ROI.
    #
    def setSubArrayMode(self):

        # Check ROI properties.
        roi_w = self.getPropertyValue("subarray_hsize")[0]
        roi_h = self.getPropertyValue("subarray_vsize")[0]
        self.properties['image_height'] = roi_h
        self.properties['image_width'] = roi_w

        # If the ROI is smaller than the entire frame turn on subarray mode
        if ((roi_w == self.max_width) and (roi_h == self.max_height)):
            self.setPropertyValue("subarray_mode", "OFF")
        else:
            self.setPropertyValue("subarray_mode", "ON")

    # startAcquisition
    #
    # Start data acquisition.
    #
    def startAcquisition(self):
        self.captureSetup()
        n_buffers = int((2.0 * 1024 * 1024 * 1024) / self.frame_bytes)
        self.number_image_buffers = n_buffers

        self.hcam_data = [HMockCamData(self.frame_x * self.frame_y, self.mock_data_max_value)
                          for i in range(1, 2)]

        self.mock_acquisiton_running = True

    # stopAcquisition
    #
    # Stop data acquisition.
    #
    def stopAcquisition(self):
        self.mock_acquisiton_running = False
        pass

    def updateIndices(self):
        pass

    # shutdown
    #
    # Close down the connection to the camera.
    #
    def shutdown(self):
        pass


class MockPZT(Driver):
    """Mock Driver for the nv401.
    """

    def __init__(self):
        super().__init__()
        self.pos = 0

    def query(self, command, *,
              send_args=(None, None), recv_args=(None, None)):
        return 'Mock PZT'

    @Feat(units='micrometer')
    def position(self):
        return self.pos

    @position.setter
    def position(self, value):
        self.pos = value

    @Action()
    def zero_position(self):
        self.pos = 0

    @Action(units='micrometer', limits=(100,))
    def moveAbsolute(self, value):
        self.pos = value

    @Action(units='micrometer')
    def moveRelative(self, value):
        self.pos = self.pos + value


class MockWebcam(Driver):

    def grab_image(self, **kwargs):
        img = np.zeros((256, 320))
        beamCenter = [int(np.random.randn() * 10 + 123),
                      int(np.random.randn() * 10 + 155)]
        img[beamCenter[0] - 10:beamCenter[0] + 10,
        beamCenter[1] - 10:beamCenter[1] + 10] = 1
        return img

    def stop(self):
        pass


class MockCameraTIS:
    def __init__(self):
        self.properties = {
            'image_height': 800,
            'image_width': 800,
            'subarray_vpos': 0,
            'subarray_hpos': 0,
            'exposure_time': 1,
            'subarray_vsize': 800,
            'subarray_hsize': 800
        }
        self.exposure = 1
        self.gain = 1
        self.brightness = 1
        self.model = 'mock'
        self.flipimage = (False, False)

    def grabFrame(self, **kwargs):
        imgsize = (800, 800)
        peakmax = 60
        noisemean = 10
        # generate image
        img = np.zeros(imgsize)
        # add a random gaussian peak sometimes
        if np.random.rand() > 0.8:
            x, y = np.meshgrid(np.linspace(0,imgsize[1],imgsize[1]), np.linspace(0,imgsize[0],imgsize[0]))
            pos = np.dstack((x, y))
            xc = (np.random.rand()*2-1)*imgsize[0]/2 + imgsize[0]/2
            yc = (np.random.rand()*2-1)*imgsize[1]/2 + imgsize[1]/2
            rv = multivariate_normal([xc, yc], [[50, 0], [0, 50]])
            img = np.random.rand()*peakmax*317*rv.pdf(pos)  #*317 to make peakval == 1
            img = img + 0.01*np.random.poisson(img)
        # add Poisson noise
        img = img + np.random.poisson(lam=noisemean, size=imgsize)
        return img

    def setPropertyValue(self, property_name, property_value):
        return property_value

    def getPropertyValue(self, property_name):
        return self.properties[property_name]

    def show_dialog(self):
        pass

    def start_live(self):
        pass

    def stop_live(self):
        pass

    def suspend_live(self):
        pass

    def setROI(self, *args, **kwargs):
        pass


class MockPCZPiezo(Driver):
    """Mock driver for the PiezoConcept Z-piezo."""

    def __init__(self):
        super().__init__()

    @Feat(read_once=True)
    def idn(self):
        """Get information of device"""
#        return self.query('INFOS')
        dummyquery = 'dummy zpiezo answer'
        return dummyquery

    def initialize(self):
        pass

    # Z-MOVEMENT

    @Feat()
    def absZ(self):
        """ Absolute Z position. """
        return 2.0

    @absZ.setter
    def absZ(self, value):
        """ Absolute Z position movement, in um. """
        print(f"Mock PCZ: setting Z position to {value} um")

    def relZ(self, value):
        """ Relative Z position movement, in um. """
        print(f"Mock PCZ: moving Z position {value} um")
        pass
        if abs(float(value)) > 0.5:
                print('Warning: Step bigger than 500 nm')

    @Action()
    def move_relZ(self, value):
        """ Relative Z position movement, in um. """
        print(f"Mock PCZ: moving Z position {value} um")
        pass
        if abs(float(value)) > 0.5:
                print('Warning: Step bigger than 500 nm')

    @Action(limits=(100,))
    def move_absZ(self, value):
        """ Absolute Z position movement, in um. """
        print(f"Mock PCZ: setting Z position to {value} um")

    # CONTROL/STATUS

    @Feat()
    def timeStep(self):
        """ Get the time between each points sent by the RAM of the USB
        interface to the nanopositioner. """
        return 1

    @timeStep.setter
    def timeStep(self, value):
        """ Set the time between each points sent by the RAM of the USB
        interface to the nanopositioner, in ms. """
        pass

    def close(self):
        pass


class MockMHXYStage(Driver):

    def __init__(self, SerialDriver=0):
        super().__init__()
        print('Simulated Marzhauser XY-stage')

    @Feat(read_once=True)
    def idn(self):
        """Get information of device"""
        return 'Marzhauser XY-stage mock'
        
    # XY-POSITION READING AND MOVEMENT

    @Feat()
    def absX(self):
        """ Read absolute X position, in um. """
        print(f"Mock MHXY: Absolute position, X.")

    @Feat()
    def absY(self):
        """ Read absolute Y position, in um. """
        print(f"Mock MHXY: Absolute position, Y.")

    @Action()
    def move_relX(self, value):
        """ Relative X position movement, in um. """
        print(f"Mock MHXY: Move relative, X: {value} um.")

    @Action()
    def move_relY(self, value):
        """ Relative Y position movement, in um. """
        print(f"Mock MHXY: Move relative, Y: {value} um.")

    @Action(limits=(100,))
    def move_absX(self, value):
        """ Absolute X position movement, in um. """
        print(f"Mock MHXY: Set position, X: {value} um.")

    @Action(limits=(100,))
    def move_absY(self, value):
        """ Absolute Y position movement, in um. """
        print(f"Mock MHXY: Set position, Y: {value} um.")

    # CONTROL/STATUS/LIMITS

    @Feat()
    def circLimit(self):
        """ Circular limits, in terms of X,Y center and radius. """
        print(f"Mock MHXY: Ask circular limits.")

    @circLimit.setter
    def circLimit(self, xpos, ypos, radius):
        """ Set circular limits, in terms of X,Y center and radius. """
        print(f"Mock MHXY: Ask circular limits, X: {xpos}, Y: {ypos}, radius: {radius}.")
     
    @Action()
    def function_press(self):
        """ Check function button presses. """
        print(f"Mock MHXY: Check button presses.")

    def close(self):
        self.finalize()


class MockRS232Driver():
    """Mock RS232 driver"""

    def __init__(self, name, settings, **kwargs):
        self._name = name
        self._settings = settings
        pass

    def query(self, arg):
        print(f"Mock: sending the following to {self._settings['port']}: {arg}")
        pass

    def initialize(self):
        pass

    def close(self):
        pass
