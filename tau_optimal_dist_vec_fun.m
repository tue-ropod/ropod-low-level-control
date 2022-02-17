function [Ttaud, taubody_sc]   = tau_optimal_dist_vec_fun( GtSl, taubody, ena_w, tau_max)
%#codegen

Nwheels = 4;

% Optimization
K = eye(2*Nwheels);
for idx = 1:Nwheels
    if ena_w(idx) == 0
        K(2*(idx-1)+1,2*(idx-1)+1) = 1000; % 1000 when wheel is defect
        K(2*(idx-1)+2,2*(idx-1)+2) = 1000;
    end
end

P = (K.'*K);
Aeq = [ GtSl  taubody];  
beq = taubody;

A = [    -eye(2*Nwheels+1);
          eye(2*Nwheels+1);];
      
b = [-tau_max*ones(2*Nwheels,1);
                             -1;
     -tau_max*ones(2*Nwheels,1);
                              0];
                                                    
f = zeros(2*Nwheels+1,1);
f(2*Nwheels+1,1) = 2*Nwheels*tau_max^2; % This guarantees that the force sacling is reduced only when neccesary

% L = chol(P, 'lower');
% Linv = L\eye(size(P,1));

% We assume diagonal and square P;
Linv = zeros(2*Nwheels+1); 
for icnt = 1:2*Nwheels
    Linv(icnt,icnt) = 1/P(icnt,icnt); % equivalent to H = P^2
end
Linv( 2*Nwheels+1 , 2*Nwheels+1 ) = 1;


x = zeros(2*Nwheels+1,1);
x = mpcqpsolver(Linv, f, A, b, Aeq, beq, false(2*(2*Nwheels+1),1), mpcqpsolverOptions());
Ttaud = x(1:2*Nwheels);
taubody_sc = 1.0-x(2*Nwheels+1);

end


