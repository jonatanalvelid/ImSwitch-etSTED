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
        params_fit = [-7.98896079e-11,
                        3.16473485e-10,
                        2.50326406e-10,
                        5.14241290e-11,
                        1.01219991e-07,
                        -7.56352531e-07,
                        -4.95637598e-07,
                        5.43228279e-02,
                        1.30569584e-03,
                        -3.55847030e+01,
                        1.04647244e-10,
                        1.35219542e-10,
                        2.62123800e-10,
                        1.85327039e-10,
                        -4.18914279e-07,
                        -2.87757055e-07,
                        -7.09441582e-07,
                        1.02333090e-03,
                        -5.39355166e-02,
                        4.21096525e+01]

        print(coords_input)
        c1 = coords_input[0]
        c2 = coords_input[1]
        x_i1 = params_fit[0]*c1**3 + params_fit[1]*c2**3 + params_fit[2]*c2*c1**2 + params_fit[3]*c1*c2**2 + params_fit[4]*c1**2 + params_fit[5]*c2**2 + params_fit[6]*c1*c2 + params_fit[7]*c1 + params_fit[8]*c2 + params_fit[9]
        x_i2 = params_fit[10]*c1**3 + params_fit[11]*c2**3 + params_fit[12]*c2*c1**2 + params_fit[13]*c1*c2**2 + params_fit[14]*c1**2 + params_fit[15]*c2**2 + params_fit[16]*c1*c2 + params_fit[17]*c1 + params_fit[18]*c2 + params_fit[19]
        return [x_i1, x_i2, 0.0]