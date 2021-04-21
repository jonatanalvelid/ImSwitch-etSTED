def wf_ps_onelens_update(coords_input):
    params_fit = [-5.71971764e-09, -2.08633482e-09, 1.71842842e-10, -3.29332562e-10, 2.27651089e-05,
                  6.09165440e-06, 6.70203240e-07, 7.31850658e-02, -6.84144526e-03, -1.29791009e+02,
                  1.85838576e-09, 2.72033328e-09, 5.12408881e-09, -6.85230704e-10, -1.24313764e-05,
                  -6.55017358e-06, -1.27917444e-05, 2.17624119e-02, -8.88191432e-02, 8.94911918e+01]
    c1 = coords_input[0]
    c2 = coords_input[1]
    x_i1 = params_fit[0]*c1**3 + params_fit[1]*c2**3 + params_fit[2]*c2*c1**2 + params_fit[3]*c1*c2**2 + params_fit[4]*c1**2 + params_fit[5]*c2**2 + params_fit[6]*c1*c2 + params_fit[7]*c1 + params_fit[8]*c2 + params_fit[9]
    x_i2 = params_fit[10]*c1**3 + params_fit[11]*c2**3 + params_fit[12]*c2*c1**2 + params_fit[13]*c1*c2**2 + params_fit[14]*c1**2 + params_fit[15]*c2**2 + params_fit[16]*c1*c2 + params_fit[17]*c1 + params_fit[18]*c2 + params_fit[19]
    return [x_i1[0], x_i2[0], 0.0]