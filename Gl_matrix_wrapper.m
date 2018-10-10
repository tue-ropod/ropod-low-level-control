function Gl = Gl_matrix_wrapper(ropod_kinmodel_param_vec, delta)

[  rW1_1,rW1_2,rW1_3,rW1_4,rW2_1,rW2_2,rW2_3,rW2_4,...
    s_w,...
    d_w, ...
    CW1_1,CW1_2,CW1_3,CW1_4,CW2_1,CW2_2,CW2_3,CW2_4 ...
    ]= extract_kinmodel_parameters(ropod_kinmodel_param_vec);

delta1 = delta(1);
delta2 = delta(2);
delta3 = delta(3);
delta4 = delta(4);

Gl = Gl_matrix_fun(CW1_1,CW1_2,CW1_3,CW1_4,CW2_1,CW2_2,CW2_3,CW2_4,d_w,delta1,delta2,delta3,delta4,rW1_1,rW1_2,rW1_3,rW1_4,rW2_1,rW2_2,rW2_3,rW2_4,s_w);