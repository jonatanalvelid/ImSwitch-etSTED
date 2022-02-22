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

        # from fit with python widget - updated 2022-02-22 - scan 20 us dwell
        params_fit = [-4.867258043104539077e-09,
                        -2.285257544220520152e-09,
                        -2.111348786336117153e-09,
                        4.377495161563817806e-09,
                        7.959991018915018614e-06,
                        1.603133659683596637e-06,
                        -1.768328713767395065e-06,
                        9.867931075865758739e-02,
                        9.966619307046945507e-04,
                        -4.100423200817824210e+01,
                        -6.713418712002687816e-10,
                        2.661493461798385969e-09,
                        1.740168489639403944e-09,
                        -7.950415100680089105e-10,
                        1.941092459440164429e-07,
                        -2.333762648318877866e-06,
                        -1.138758151848054817e-06,
                        1.258471432774147403e-03,
                        -1.012687158228265938e-01,
                        4.297666571852118977e+01
                        ]

        print(coords_input)
        c1 = coords_input[0]
        c2 = coords_input[1]
        x_i1 = params_fit[0]*c1**3 + params_fit[1]*c2**3 + params_fit[2]*c2*c1**2 + params_fit[3]*c1*c2**2 + params_fit[4]*c1**2 + params_fit[5]*c2**2 + params_fit[6]*c1*c2 + params_fit[7]*c1 + params_fit[8]*c2 + params_fit[9]
        x_i2 = params_fit[10]*c1**3 + params_fit[11]*c2**3 + params_fit[12]*c2*c1**2 + params_fit[13]*c1*c2**2 + params_fit[14]*c1**2 + params_fit[15]*c2**2 + params_fit[16]*c1*c2 + params_fit[17]*c1 + params_fit[18]*c2 + params_fit[19]
        print([x_i1, x_i2])
        return [x_i1, x_i2, 0.0]