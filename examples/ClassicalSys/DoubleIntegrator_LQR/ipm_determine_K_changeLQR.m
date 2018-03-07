function [K, results] = ipm_determine_K_changeLQR(xt, params, dt,Q,R)
% creates the PD gains for the DPM model
% first it creates discrete linear dynamics for the model over the
% trajectory 
%   x_{k+1} = x_k + A_k * x_k
% by using dfdx to create A_k
% then uses MATLAB's lqr function to determine the controller gains
% Q is set to I
% R is set to 0.01I
% N is set to 0

%initialize A matrix
A = cell(size(xt,1),1);
dfdx_over_time = zeros(size(xt,1),2);

for i=1:size(xt,1)
    dfdx = (dfdx_m1(xt(i,:), params));
    A{i} = [0 1;
           dfdx];
end

%PATRICK 1/12/2017
B = [0;1];
% Q = [.9 0; 0 0.5];
% % PATRICK playing with LQR gains
% % R = [0.01];
% R = [0.01];

%May need to change dimensions of xt
K = LQR_timevarying(A, B, Q, R, xt(:,1), dt);
results = [];
end