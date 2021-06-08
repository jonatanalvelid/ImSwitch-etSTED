def wf_ps_propwf(coords_input, *args, **kwargs):
        # from fit with ipynb with imagej-determined coords
        #params_fit = [  3.93752742e-10,   7.13015278e-10,   3.50209221e-10,
        #    -7.09835640e-11,  -6.68130980e-07,  -1.54426116e-06,
        #    -7.86134317e-08,   5.41977917e-02,   1.09324417e-03,
        #    -3.71342520e+01,  -4.08235816e-10,  -1.35179220e-09,
        #    -1.61874836e-09,   7.60458086e-10,   2.04502348e-06,
        #    2.62879875e-06,   8.50711143e-07,  -1.89490355e-03,
        #    5.25656872e-02,  -4.33482929e+01] 

        # from fit with python widget
        params_fit = [ -1.38019961e-10,   5.52727897e-11,  -8.58828367e-11,   1.17656125e-11,
                        5.98268175e-07,  -1.02246334e-07,   4.51340319e-08,   5.36818026e-02,
                        2.00394793e-04,  -3.69571391e+01,  -4.36480113e-10,  -2.18662452e-10,
                        -6.18790918e-10,  -1.24206936e-10,   1.47463500e-06,   6.42903595e-07,
                        1.08862802e-06,  -1.76809701e-03,   5.34696580e-02,  -4.32596346e+01]
        c1 = coords_input[0]
        c2 = coords_input[1]
        x_i1 = params_fit[0]*c1**3 + params_fit[1]*c2**3 + params_fit[2]*c2*c1**2 + params_fit[3]*c1*c2**2 + params_fit[4]*c1**2 + params_fit[5]*c2**2 + params_fit[6]*c1*c2 + params_fit[7]*c1 + params_fit[8]*c2 + params_fit[9]
        x_i2 = params_fit[10]*c1**3 + params_fit[11]*c2**3 + params_fit[12]*c2*c1**2 + params_fit[13]*c1*c2**2 + params_fit[14]*c1**2 + params_fit[15]*c2**2 + params_fit[16]*c1*c2 + params_fit[17]*c1 + params_fit[18]*c2 + params_fit[19]
        return [x_i1[0], x_i2[0], 0.0]