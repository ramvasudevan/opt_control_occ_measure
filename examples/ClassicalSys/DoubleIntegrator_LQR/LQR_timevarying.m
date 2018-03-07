function K = LQR_timevarying(A,B,Q,R,t,dt)
% This function computes a time-varying gain matrix K(t)
% given cells A{}, B{}, Q{}, and R{} and a time vector
%store each element of the gain matrix in a cell K{1}, K{2}, etc

% 10/19/2016
%
% first, solve for P(t) using Heun's method
% PATRICK scaled dt

P=cell(length(t),1); P_intermed=cell(length(t),1); K=cell(length(t),1);
P{1}=Q;
%
for i=1:length(t)-1
    %initial prediction P_intermed made with Euler's method (not very
    %accurate)
    P_intermed{i+1}=P{i}+dt*P_dot(A,B,Q,R,P{i});
    %final approximation of P{i+1} is achieved by using an average of the
    %two slopes from P_i and P_intermed
    P{i+1}=P{i}+dt/2*(P_dot(A,B,Q,R,P{i})+...
        P_dot(A,B,Q,R,P_intermed{i+1}));
    %Now determine K{i}
    K{i}=R\B'*P{i};
end
%K{end} isn't defined in the loop (only goes to length(t)-1)
K{end}=R\B'*P{end};
end