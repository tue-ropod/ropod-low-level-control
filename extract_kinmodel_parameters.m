function [  rW1_1,rW1_2,rW1_3,rW1_4,rW2_1,rW2_2,rW2_3,rW2_4,...
    s_w,...
    d_w, ...
    CW1_1,CW1_2,CW1_3,CW1_4,CW2_1,CW2_2,CW2_3,CW2_4 ...
    ]= extract_kinmodel_parameters(ropod_kinmodel_param_vec)
%EXTRACT_KINMODEL_PARAMETERS Summary of this function goes here
%   Detailed explanation goes here

Nwheels = 4;
nv_size = 2*Nwheels;
i_ini = 1;
i_end = i_ini-1+nv_size;
r_w_vec = ropod_kinmodel_param_vec(i_ini:i_end);
nv_size = 1;
i_ini = i_end+1;
i_end = i_ini-1+nv_size;
s_w_vec = ropod_kinmodel_param_vec(i_ini:i_end);
nv_size = 1;
i_ini = i_end+1;
i_end = i_ini-1+nv_size;
d_w_vec = ropod_kinmodel_param_vec(i_ini:i_end);
nv_size = 2*Nwheels;
i_ini = i_end+1;
i_end = i_ini-1+nv_size;
C_SW_vec = ropod_kinmodel_param_vec(i_ini:i_end);

rW1_1 = r_w_vec(1);
rW2_1 = r_w_vec(2);
rW1_2 = r_w_vec(3);
rW2_2 = r_w_vec(4);
rW1_3 = r_w_vec(5);
rW2_3 = r_w_vec(6);
rW1_4 = r_w_vec(7);
rW2_4 = r_w_vec(8);

s_w = s_w_vec(1);
d_w = d_w_vec(1);


CW1_1 = C_SW_vec(1);
CW2_1 = C_SW_vec(2);
CW1_2 = C_SW_vec(3);
CW2_2 = C_SW_vec(4);
CW1_3 = C_SW_vec(5);
CW2_3 = C_SW_vec(6);
CW1_4 = C_SW_vec(7);
CW2_4 = C_SW_vec(8);


end

