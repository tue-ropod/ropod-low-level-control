function S = S_matrix()

Nwheels = 4;

% Torques are applied directly in the varphi variable coordinate.
% This definition of S is be used for modelling purposes
S = [zeros(3+Nwheels,2*Nwheels);
     eye(2*Nwheels)];

end

