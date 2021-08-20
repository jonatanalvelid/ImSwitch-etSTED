def wf_800_scan_80(coords_input, *args, **kwargs):
        # from fit with ipynb with imagej-determined coords
        #params_fit = [  3.93752742e-10,   7.13015278e-10,   3.50209221e-10,
        #    -7.09835640e-11,  -6.68130980e-07,  -1.54426116e-06,
        #    -7.86134317e-08,   5.41977917e-02,   1.09324417e-03,
        #    -3.71342520e+01,  -4.08235816e-10,  -1.35179220e-09,
        #    -1.61874836e-09,   7.60458086e-10,   2.04502348e-06,
        #    2.62879875e-06,   8.50711143e-07,  -1.89490355e-03,
        #    5.25656872e-02,  -4.33482929e+01] 

        # from fit with python widget - ham cam - updated 2021-08-20
        params_fit = [-4.196053726108963979e-09,
                        1.546524272067301727e-09,
                        -6.747491056256467578e-10,
                        7.477303045766036493e-10,
                        6.524545162988059792e-06,
                        -1.784650942172131030e-06,
                        -9.657677692405839113e-08,
                        9.921448313680568398e-02,
                        2.018178473581374883e-03,
                        -4.180434672741007063e+01,
                        -2.374468827657591110e-09,
                        -3.218955000975820911e-10,
                        1.087349185984601523e-09,
                        5.605192088547700347e-10,
                        2.915897720230791589e-06,
                        3.147111718919463429e-07,
                        -1.998450762323992130e-06,
                        4.862342179788697026e-04,
                        -1.016728760337577142e-01,
                        3.987306198723644002e+01
                        ]

        print(coords_input)
        c1 = coords_input[0]
        c2 = coords_input[1]
        x_i1 = params_fit[0]*c1**3 + params_fit[1]*c2**3 + params_fit[2]*c2*c1**2 + params_fit[3]*c1*c2**2 + params_fit[4]*c1**2 + params_fit[5]*c2**2 + params_fit[6]*c1*c2 + params_fit[7]*c1 + params_fit[8]*c2 + params_fit[9]
        x_i2 = params_fit[10]*c1**3 + params_fit[11]*c2**3 + params_fit[12]*c2*c1**2 + params_fit[13]*c1*c2**2 + params_fit[14]*c1**2 + params_fit[15]*c2**2 + params_fit[16]*c1*c2 + params_fit[17]*c1 + params_fit[18]*c2 + params_fit[19]
        print([x_i1, x_i2])
        return [x_i1, x_i2, 0.0]