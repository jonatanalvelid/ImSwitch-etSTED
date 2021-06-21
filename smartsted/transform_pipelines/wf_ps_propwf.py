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
        params_fit = [-8.51686318e-10,
                        1.54586394e-11,
                        4.63734493e-10,
                        2.43382323e-11,
                        1.52352769e-06,
                        4.62728531e-08,
                        -6.70523443e-07,
                        5.36009635e-02,
                        7.52936652e-04,
                        -3.53997840e+01,
                        1.46320746e-11,
                        2.32447544e-10,
                        3.58832611e-10,
                        -6.31277734e-11,
                        -3.92425967e-07,
                        -3.48828877e-07,
                        -5.26953157e-07,
                        1.09860425e-03,
                        -5.39450936e-02,
                        4.21383696e+01]


        print(coords_input)
        c1 = coords_input[0]
        c2 = coords_input[1]
        x_i1 = params_fit[0]*c1**3 + params_fit[1]*c2**3 + params_fit[2]*c2*c1**2 + params_fit[3]*c1*c2**2 + params_fit[4]*c1**2 + params_fit[5]*c2**2 + params_fit[6]*c1*c2 + params_fit[7]*c1 + params_fit[8]*c2 + params_fit[9]
        x_i2 = params_fit[10]*c1**3 + params_fit[11]*c2**3 + params_fit[12]*c2*c1**2 + params_fit[13]*c1*c2**2 + params_fit[14]*c1**2 + params_fit[15]*c2**2 + params_fit[16]*c1*c2 + params_fit[17]*c1 + params_fit[18]*c2 + params_fit[19]
        return [x_i1, x_i2, 0.0]