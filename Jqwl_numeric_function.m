function [v] = Jqwl_numeric_function(ropod_kinmodel_param_vec, delta, dvarphi, ena_w)

Nwheels = 4;
% v = zeros(3,1);

[  rW1_1,rW1_2,rW1_3,rW1_4,rW2_1,rW2_2,rW2_3,rW2_4,...
    s_w,...
    d_w, ...
    CW1_1,CW1_2,CW1_3,CW1_4,CW2_1,CW2_2,CW2_3,CW2_4 ...
    ]= extract_kinmodel_parameters(ropod_kinmodel_param_vec);

CW = [[CW1_1 ; CW2_1], [CW1_2 ; CW2_2], [CW1_3 ; CW2_3], [CW1_4 ; CW2_4]];
r_w = [[rW1_1 ; rW2_1], [rW1_2 ; rW2_2], [rW1_3 ; rW2_3], [rW1_4 ; rW2_4]];


%% First compute the matrix from v to W_wh

% Gi = zeros([2 3]);
G = zeros(2*Nwheels,3);
M = zeros(2*Nwheels,2*Nwheels);

for i=1:Nwheels    
    % To obtain pivot velocities in Robot coordinates, add vectors of
    % translations and movement due to angular velocity.
    Gi = [1 0 -CW(2,i);
                 0 1  CW(1,i)];
    
    % Rotation to obtain pivot Wi velocity vector in wheel coordinates
    Gi = [   cos(-delta(i))     -sin(-delta(i)); ...
                    sin(-delta(i))     cos(-delta(i))]*Gi;    
    
    
    % From pivot's Wi velocity to wheel's velocity vector
    Gi = [   1, -d_w/(2*s_w);  ...
                    1,  d_w/(2*s_w)]* Gi;    
    
    % Compute wheel's angular velocity
    Gi =  diag(1./r_w(:,i))*Gi;
    % Now lets compute 
    
    G(2*i-1:2*i,:) = Gi;    
    M(2*i-1:2*i,2*i-1:2*i)=diag([ena_w(i) ena_w(i)]);
end     

v = pinv(M*G)*(M*dvarphi);





end

