function [v] = Jqwl_numeric_function(ropod_kinmodel_param_vec, delta, dvarphi, ena_w)

Nwheels = 4;

[  rW1_1,rW1_2,rW1_3,rW1_4,rW2_1,rW2_2,rW2_3,rW2_4,...
    s_w,...
    d_w, ...
    CW1_1,CW1_2,CW1_3,CW1_4,CW2_1,CW2_2,CW2_3,CW2_4 ...
    ]= extract_kinmodel_parameters(ropod_kinmodel_param_vec);

CW = [[CW1_1 ; CW2_1], [CW1_2 ; CW2_2], [CW1_3 ; CW2_3], [CW1_4 ; CW2_4]];
r_w = [[rW1_1 ; rW2_1], [rW1_2 ; rW2_2], [rW1_3 ; rW2_3], [rW1_4 ; rW2_4]];

vR_W    = zeros(2,Nwheels); % vectors (columns) of Wi velocities in Robot coordinates. 
vW_W    = zeros(2,Nwheels); % vectors (columns) of Wi velocities in Wheel pair coordinates.
W_wh    = zeros(2,Nwheels); % vdvarphi: vectors (columns) of wheels velocities per Wheel pair. First row is left wheel.
V_wh    = zeros(2,Nwheels);  % vectors (columns) of wheels velocities in m/s per Wheel pair. First row is left wheel.

vR_R = [0;0];
wR_R = 0;

N_act_wheels = sum(ena_w)/2;  %  We assume that ena_w come in pairs
ena_sw = zeros(1,Nwheels);
for i=1:Nwheels    
    W_wh(1,i) = dvarphi(2*i-1);
    W_wh(2,i) = dvarphi(2*i);
    V_wh(:,i) = r_w(:,i).* W_wh(:,i);
    ena_sw(i) = ena_w(2*i-1)*ena_w(2*i);
    
    % From wheel's velocity to pivot point Wi velocity vector
    vW_W(:,i) = [   1/2     1/2 ;  ...
                    -s_w/d_w    s_w/d_w]* V_wh(:,i);

    % Rotation to obtain pivot Wi velocity vector in robot coordinates
    vR_W(:,i) = [   cos(delta(i))     -sin(delta(i)); ...
                    sin(delta(i))     cos(delta(i))]*vW_W(:,i);
               
    vR_R = vR_R +  1/N_act_wheels * vR_W(:,i) * ena_sw(i);
    wR_R = wR_R + 1/N_act_wheels / sum(CW(:,i).^2) * [-CW(2,i) CW(1,i)] * vR_W(:,i) * ena_sw(i);
end        

v = [vR_R; wR_R];










end

