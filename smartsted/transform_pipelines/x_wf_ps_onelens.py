def x_wf_ps_onelens(coords_input):
    params_fit = [-1.49214867e-05, 3.06270078e-07, 2.65272898e-07, -9.82871829e-06, 3.69354916e-03,
                    8.12142648e-04, 1.10616761e-03, 6.98015969e-01, -8.94239771e-02, -7.40580824e+01,
                    1.94301147e-06, -1.60325351e-05, -1.09915956e-05, 2.15341641e-07, 7.15914502e-05,
                    2.93541915e-03, 1.80452962e-03, -5.99629797e-02, 7.78267334e-01, -4.86740835e+01]
    c1 = coords_input[0]/16
    c2 = coords_input[1]/16
    x_i1 = params_fit[0]*c1**3 + params_fit[1]*c2**3 + params_fit[2]*c2*c1**2 + params_fit[3]*c1*c2**2 + params_fit[4]*c1**2 + params_fit[5]*c2**2 + params_fit[6]*c1*c2 + params_fit[7]*c1 + params_fit[8]*c2 + params_fit[9]
    x_i2 = params_fit[10]*c1**3 + params_fit[11]*c2**3 + params_fit[12]*c2*c1**2 + params_fit[13]*c1*c2**2 + params_fit[14]*c1**2 + params_fit[15]*c2**2 + params_fit[16]*c1*c2 + params_fit[17]*c1 + params_fit[18]*c2 + params_fit[19]
    return [x_i1[0], x_i2[0], 0.0]