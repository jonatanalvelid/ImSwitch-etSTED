def wf_800_scan_80(coords_input, *args, **kwargs):
        "Pre-calibrated coordinate transform for etSTED in the REDsted"
        # from fit with ipynb with imagej-localized coords
        #params_fit = [  3.93752742e-10,   7.13015278e-10,   3.50209221e-10,
        #    -7.09835640e-11,  -6.68130980e-07,  -1.54426116e-06,
        #    -7.86134317e-08,   5.41977917e-02,   1.09324417e-03,
        #    -3.71342520e+01,  -4.08235816e-10,  -1.35179220e-09,
        #    -1.61874836e-09,   7.60458086e-10,   2.04502348e-06,
        #    2.62879875e-06,   8.50711143e-07,  -1.89490355e-03,
        #    5.25656872e-02,  -4.33482929e+01] 

        # from fit with python widget - updated 2021-08-27 - scan 20 us dwell
        params_fit = [-1.188033331880831518e-09,
                        1.097083366010643169e-09,
                        -5.273500980338464584e-10,
                        -1.744396884715451083e-10,
                        2.612140597226462025e-06,
                        -8.785963639722968196e-07,
                        3.814454330947862371e-07,
                        1.005218160888972645e-01,
                        1.545471775768740130e-03,
                        -4.180114799086727118e+01,
                        1.268104888235232562e-09,
                        -7.930701548649460406e-11,
                        2.860267189879632174e-09,
                        -4.779925076064363007e-10,
                        -2.890527398129946892e-06,
                        7.262138637063089313e-07,
                        -2.423884516814759662e-06,
                        2.900644474070668971e-03,
                        -1.019673565794493308e-01,
                        3.971132215492339412e+01
                        ]

        print(coords_input)
        c1 = coords_input[0]
        c2 = coords_input[1]
        x_i1 = params_fit[0]*c1**3 + params_fit[1]*c2**3 + params_fit[2]*c2*c1**2 + params_fit[3]*c1*c2**2 + params_fit[4]*c1**2 + params_fit[5]*c2**2 + params_fit[6]*c1*c2 + params_fit[7]*c1 + params_fit[8]*c2 + params_fit[9]
        x_i2 = params_fit[10]*c1**3 + params_fit[11]*c2**3 + params_fit[12]*c2*c1**2 + params_fit[13]*c1*c2**2 + params_fit[14]*c1**2 + params_fit[15]*c2**2 + params_fit[16]*c1*c2 + params_fit[17]*c1 + params_fit[18]*c2 + params_fit[19]
        print([x_i1, x_i2])
        return [x_i1, x_i2, 0.0]