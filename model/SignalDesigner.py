# -*- coding: utf-8 -*-
"""
Created on Thu Apr  9 09:20:14 2020

@author: andreas.boden
"""

import numpy as np

# try:
#    from .errors import InvalidChildClassError, IncompatibilityError
# except ModuleNotFoundError:
from .errors import InvalidChildClassError


class SignalDesignerFactory:
    """Factory class for creating a SignalDesigner object. Factory checks
    that the new object is compatible with the parameters that will we 
    be sent to its make_signal method."""

    def __new__(cls, setupInfo, configKeyName):
        scanDesignerName = getattr(setupInfo.designers, configKeyName)

        #        SignalDesigner = super().__new__(cls, 'SignalDesigner.'+scanDesignerName)
        signalDesigner = globals()[scanDesignerName]()
        if signalDesigner.isValidSignalDesigner():
            return signalDesigner


class SignalDesigner:
    """Parent class for any type of SignaDesigner. Any child should define
    self._expected_parameters and its own make_signal method."""

    def __init__(self):

        self.lastSignal = None
        self.lastParameterDict = None

        self._expectedParameters = None

        # Make non-overwritable functions
        self.isValidSignalDesigner = self.__isValidSignalDesigner
        self.parameterCompatibility = self.__parameterCompatibility

    @property
    def expectedParameters(self):
        if self._expectedParameters is None:
            raise ValueError('Value "%s" is not defined')
        else:
            return self._expectedParameters

    def __isValidSignalDesigner(self):
        if self._expectedParameters is None:
            raise InvalidChildClassError('Child of SignalDesigner should define \
                                 "self.expected_parameters" in __init__.')
        else:
            return True

    def make_signal(self, parameterDict, setupInfo):
        """ Method to be defined by child. Should return a dictionary with 
        {'target': signal} pairs. """
        raise NotImplementedError("Method not implemented in child")

    def __parameterCompatibility(self, parameterDict):
        """ Method to check the compatibility of parameter 'parameterDict'
        and the expected parameters of the object. """
        expected = set(self._expectedParameters)
        incoming = set([*parameterDict])

        return expected == incoming


class BetaStageScanDesigner(SignalDesigner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._expectedParameters = ['Targets[x]',
                                    'Sizes[x]',
                                    'Step_sizes[x]',
                                    'Start[x]',
                                    'Sequence_time_seconds',
                                    'Sample_rate',
                                    'Return_time_seconds']

    def make_signal(self, parameterDict, setupInfo, returnFrames=False):

        if not self.parameterCompatibility(parameterDict):
            print([*parameterDict])
            print(self._expectedParameters)
            print('Stage scan parameters seem incompatible, this error should not be since this should be checked at program start-up')
            return None

        convFactors = [positioner.managerProperties['conversionFactor']
                       for positioner in setupInfo.positioners.values()]

        # Retrieve sizes
        [fast_axis_size, middle_axis_size, slow_axis_size] = \
            [(parameterDict['Sizes[x]'][i] / convFactors[i]) for i in range(3)]

        # Retrieve step sized
        [fast_axis_step_size, middle_axis_step_size, slow_axis_step_size] = \
            [(parameterDict['Step_sizes[x]'][i] / convFactors[i]) for i in range(3)]

        # Retrive starting position
        [fast_axis_start, middle_axis_start, slow_axis_start] = \
            [(parameterDict['Start[x]'][i] / convFactors[i]) for i in range(3)]

        fast_axis_positions = 1 + np.int(np.ceil(fast_axis_size / fast_axis_step_size))
        middle_axis_positions = 1 + np.int(np.ceil(middle_axis_size / middle_axis_step_size))
        slow_axis_positions = 1 + np.int(np.ceil(slow_axis_size / slow_axis_step_size))

        sequenceSamples = parameterDict['Sequence_time_seconds'] * parameterDict['Sample_rate']
        returnSamples = parameterDict['Return_time_seconds'] * parameterDict['Sample_rate']
        if not sequenceSamples.is_integer():
            print('WARNING: Non-integer number of sequence samples, rounding up')
        sequenceSamples = np.int(np.ceil(sequenceSamples))
        if not returnSamples.is_integer():
            print('WARNING: Non-integer number of return samples, rounding up')
        returnSamples = np.int(np.ceil(returnSamples))

        # Make fast axis signal
        rampSamples = fast_axis_positions * sequenceSamples
        lineSamples = rampSamples + returnSamples

        rampSignal = self.__makeRamp(fast_axis_start, fast_axis_size, rampSamples)
        returnRamp = self.__smoothRamp(fast_axis_size, fast_axis_start, returnSamples)
        fullLineSignal = np.concatenate((rampSignal, returnRamp))

        fastAxisSignal = np.tile(fullLineSignal, middle_axis_positions * slow_axis_positions)
        # Make middle axis signal
        colSamples = middle_axis_positions * lineSamples
        colValues = self.__makeRamp(middle_axis_start, middle_axis_size, middle_axis_positions)
        fullSquareSignal = np.zeros(colSamples)
        for s in range(middle_axis_positions):
            fullSquareSignal[s * lineSamples: s * lineSamples + rampSamples] = colValues[s]

            try:
                fullSquareSignal[s * lineSamples + rampSamples:(s + 1) * lineSamples] = \
                    self.__smoothRamp(colValues[s], colValues[s + 1], returnSamples)
            except IndexError:
                fullSquareSignal[s * lineSamples + rampSamples:(s + 1) * lineSamples] = \
                    self.__smoothRamp(colValues[s], middle_axis_start, returnSamples)

        middleAxisSignal = np.tile(fullSquareSignal, slow_axis_positions)

        # Make slow axis signal
        sliceSamples = slow_axis_positions * colSamples
        sliceValues = self.__makeRamp(slow_axis_start, slow_axis_size, slow_axis_positions)
        fullCubeSignal = np.zeros(sliceSamples)
        for s in range(slow_axis_positions):
            fullCubeSignal[s * colSamples:(s + 1) * colSamples - returnSamples] = sliceValues[s]

            try:
                fullCubeSignal[(s + 1) * colSamples - returnSamples:(s + 1) * colSamples] = \
                    self.__smoothRamp(sliceValues[s], sliceValues[s + 1], returnSamples)
            except IndexError:
                fullCubeSignal[(s + 1) * colSamples - returnSamples:(s + 1) * colSamples] = \
                    self.__smoothRamp(sliceValues[s], slow_axis_start, returnSamples)
        slowAxisSignal = fullCubeSignal

        sig_dict = {parameterDict['Targets[x]'][0]: fastAxisSignal,
                    parameterDict['Targets[x]'][1]: middleAxisSignal,
                    parameterDict['Targets[x]'][2]: slowAxisSignal}

        if not returnFrames:
            return sig_dict
        else:
            return sig_dict, [fast_axis_positions, middle_axis_positions, slow_axis_positions]

    def __makeRamp(self, start, end, samples):
        return np.linspace(start, end, num=samples)

    def __smoothRamp(self, start, end, samples):
        curve_half = 0.6
        n = np.int(np.floor(curve_half * samples))
        x = np.linspace(0, np.pi / 2, num=n, endpoint=True)
        signal = start + (end - start) * np.sin(x)
        signal = np.append(signal, end * np.ones(int(np.ceil((1 - curve_half) * samples))))
        return signal


class BetaTTLCycleDesigner(SignalDesigner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._expectedParameters = ['Targets[x]',
                                    'TTLStarts[x,y]',
                                    'TTLEnds[x,y]',
                                    'Sequence_time_seconds',
                                    'Sample_rate']

    def make_signal(self, parameterDict, setupInfo):

        if not self.parameterCompatibility(parameterDict):
            print('TTL parameters seem incompatible, this error should not be \
                  since this should be checked at program start-up')
            return None

        targets = parameterDict['Targets[x]']
        sampleRate = parameterDict['Sample_rate']
        cycleSamples = parameterDict['Sequence_time_seconds'] * sampleRate
        if not cycleSamples.is_integer():
            print('WARNING: Non-integer number of sequence samples, rounding up')
        cycleSamples = np.int(np.ceil(cycleSamples))
        signalDict = {}
        tmpSigArr = np.zeros(cycleSamples, dtype='bool')
        for i, target in enumerate(targets):
            tmpSigArr[:] = False
            for j, start in enumerate(parameterDict['TTLStarts[x,y]'][i]):
                startSamp = np.int(np.round(start * sampleRate))
                endSamp = np.int(np.round(parameterDict['TTLEnds[x,y]'][i][j] * sampleRate))
                tmpSigArr[startSamp:endSamp] = True

            signalDict[target] = np.copy(tmpSigArr)
        return signalDict


class GalvoScanDesigner(SignalDesigner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        self._expectedParameters = ['axis_scanner',
                                    'axis_length',
                                    'axis_center',
                                    'pixel_size',
                                    'pixel_dwelltime',
                                    'time_step',
                                    'vel_max',
                                    'acc_max',
                                    'sample_rate']

        self.__settlingtime = 50e3  # arbitrary for now
        self.__paddingtime = 200e3  # arbitrary for now

    def make_signal(self, parameterDict, setupInfo):
        convFactors = [positioner.managerProperties['conversionFactor']
                for positioner in setupInfo.positioners.values()]

        # fast axis signal
        fast_pos = __generate_smooth_scan(parameterDict)

        # slow axis signal
        axis_reps = __get_axis_reps(fast_pos)
        slow_pos = __generate_step_scan(parameterDict, axis_reps)

        # pad all signals
        fastAxisSignal, slowAxisSignal = __zero_padding(parameterDict, fast_pos, slow_pos)

        sig_dict = {parameterDict['axis_scanner'][0]: fastAxisSignal,
                    parameterDict['axis_scanner'][1]: slowAxisSignal}       
        return sig_dict

    def __generate_smooth_scan(parameterDict):
        """ Generate a smooth scanning curve with spline interpolation """ 
        n_lines = int(parameterDict['axis_length'][1]/parameterDict['pixel_size'][1])  # number of lines
        t_settling = self.__settlingtime/parameterDict['time_step']  # initial settling time before first line
        # generate 1 period of curve
        curve_poly, time_fix, pos_fix = __linescan_poly(parameterDict)
        # calculate number of evaluation points for a line for decided timestep
        n_eval = int(time_fix[-1]/parameterDict['time_step'])
        # generate multiline curve for the whole scan
        pos = __generate_smooth_multiline(curve_poly, time_fix, pos_fix, n_eval, n_lines)
        # add missing start and end piece
        pos_ret = __add_start_end(pos, pos_fix, t_settling)
        return pos_ret

    def __generate_step_scan(parameterDict, axis_steps):
        """ Generate a step-function scanning curve """
        l_scan = parameterDict['axis_length'][1]
        c_scan = parameterDict['axis_center'][1]
        n_lines = int(parameterDict['axis_length'][1]/parameterDict['pixel_size'][1])
        # create linspace for positions of interest
        positions = np.linspace(l_scan/n_lines, l_scan, n_lines) - l_scan/(n_lines*2)
        # repeat each middle element a number of times equal to the length of that between the faster axis repetitions
        pos = np.repeat(positions, np.diff(axis_steps))
        # shift center of axis to center of image
        pos = pos - l_scan/2
        # add y_center to all values  
        pos_ret = pos + c_scan
        return pos_ret

    def __get_axis_reps(pos):
        """ Get the time of the steps taken on the axis provided (fast axis) """
        # get the positions of the velocity sign changes
        possign = np.sign(np.diff(pos))
        signchangevel = ((np.roll(possign, 1) - possign) != 0).astype(int)
        # get all velocity signchanges
        signchanges = np.where(signchangevel == 1)[0]
        # get the negative velocity sign changes for the start of the lines
        axis_reps = signchanges[::2]
        # append the last position, to calculate the length of the last step
        axis_reps = np.append(axis_reps, len(pos))
        return axis_reps

    def __linescan_poly(parameterDict):
        """ Generate a Bernstein piecewise polynomial for a smooth one-line scanning curve,
        from the acquisition parameter settings, using piecewise spline interpolation """
        l_scan = parameterDict['axis_length'][0]
        c_scan = parameterDict['axis_center'][0]
        v_scan = parameterDict['pixel_size'][0]/parameterDict['pixel_dwelltime']
        v_max = parameterDict['vel_max'][0]*1e-3
        a_max = parameterDict['acc_max'][0]*1e-6
        dt_fix = parameterDict['time_step']  # time between two fix points where the acceleration changes (infinite jerk)
        
        # positions at fixed points
        p1 = c_scan
        p2 = p2p = p1 + l_scan/2
        t_deacc = (v_scan+v_max)/a_max
        d_deacc = v_scan*t_deacc + 0.5*(-a_max)*t_deacc**2
        p3 = p3p = p2 + d_deacc
        p4 = p4p = c_scan - (p3 - c_scan)
        p5 = p5p = p1 - l_scan/2
        p6 = p1
        pos = [p1, p2, p2p, p3, p3p, p4, p4p, p5, p5p, p6]
        
        # time at fixed points
        t1 = 0
        t_scanline = l_scan/v_scan
        t2 = t1 + t_scanline/2
        t2p = t2 + dt_fix
        t3 = t2 + t_deacc
        t3p = t3 + dt_fix
        t4 = t3 + abs(p4 - p3)/v_max
        t4p = t4 + dt_fix
        t_acc = t_deacc
        t5 = t4 + t_acc
        t5p = t5 + dt_fix
        t6 = t5 + t_scanline/2
        time = [t1, t2, t2p, t3, t3p, t4, t4p, t5, t5p, t6]
        
        # velocity at fixed points
        v1 = v_scan
        v2 = v2p = v_scan
        v3 = v3p = v4 = v4p = -v_max
        v5 = v5p = v6 = v_scan
        vel = [v1, v2, v2p, v3, v3p, v4, v4p, v5, v5p, v6]
        
        # acceleration at fixed points
        a1 = a2 = 0
        a2p = a3 = -a_max
        a3p = a4 = 0
        a4p = a5 = a_max
        a5p = a6 = 0
        acc = [a1, a2, a2p, a3, a3p, a4, a4p, a5, a5p, a6]
        
        # if p3 is already past the center of the scan it means that the max_velocity was never reached
        # in this case, remove two fixed points, and change the values to the curr. vel and time in the
        # middle of the flyback
        if p3 <= c_scan:
            t_mid = np.roots([-a_max/2, v_scan, p2-c_scan])[0]
            v_mid = -a_max*t_mid + v_scan
            del pos[5:7]
            del vel[5:7]
            del acc[5:7]
            del time[5:7]
            pos[3:5] = [c_scan, c_scan]
            vel[3:5] = [v_mid, v_mid]
            acc[3:5] = [-a_max, a_max]
            time[3] = time[2] + t_mid
            time[4] = time[3] + dt_fix
            time[5] = time[3] + t_mid
            time[6] = time[5] + dt_fix
            time[7] = time[5] + t_scanline/2
        
        # generate Bernstein polynomial with piecewise spline interpolation with the fixed points
        # give positions, velocity, acceleration, and time of fixed points
        yder = np.array([pos, vel, acc]).T.tolist()
        bpoly = BPoly.from_derivatives(time, yder) # bpoly time unit: µs
        
        # return polynomial, that can be evaluated at any timepoints you want
        # return fixed points position and time
        return bpoly, time, pos

    def __generate_smooth_multiline(pos_bpoly, time_fix, pos_fix, n_eval, n_lines):
        """ Generate a smooth multiline curve by evaluating the polynomial with the clock frequency used and copying it """ 
        # get evaluation times for one line
        x_eval = linspace(0, time_fix[-1], n_eval)
        # evaluate polynomial
        x_bpoly = pos_bpoly(x_eval)
        pos_ret = []
        # concatenate for number of lines in scan
        for i in range(n_lines-1):
            pos_ret = np.append(pos_ret, x_bpoly[:-1])
        return pos_ret

    def __add_start_end(pos, pos_fix, t_settling):
        """ Add start and end half-lines to smooth scanning curve """
        # generate three pieces, two before and one after, to be concatenated to the given positions array
        pos_pre1 = np.repeat(np.min(pos),t_settling)
        pos_pre2 = pos[np.where(pos==np.min(pos))[0][-1]:]
        pos_post1 = pos[:np.argmin(abs(pos[:np.where(pos==np.max(pos))[0][0]]-pos_fix[2]))]
        pos_ret = pos
        pos_ret = np.append(pos_pre2, pos_ret)
        pos_ret = np.append(pos_pre1, pos_ret)
        pos_ret = np.append(pos_ret, pos_post1)
        return pos_ret

    def __zero_padding(parameterDict, pos1, pos2):
        """ Pad zeros to the end of two scanning curves, for initial and final settling of galvos """
        padlen = int(self.__paddingtime / parameterDict['time_step'])
        # check that the length of pos1 and pos2 are identical
        padlen1 = np.array([padlen,padlen])
        padlen2 = np.array([padlen,padlen])
        lendiff = abs(len(pos1)-len(pos2))
        # if not equal, add to the correct padding length to make them equal
        if lendiff != 0:
            if lendiff > 0:
                padlen2 = padlen2 + np.array([0,lendiff])
                print(padlen2)
            elif lendiff < 0:
                padlen1 = padlen1 + np.array([0,lendiff])
                print(padlen1)
        # pad position arrays
        pos_ret1 = np.pad(pos1, padlen1, 'constant', constant_values=0)
        pos_ret2 = np.pad(pos2, padlen2, 'constant', constant_values=0)
        return pos_ret1, pos_ret2