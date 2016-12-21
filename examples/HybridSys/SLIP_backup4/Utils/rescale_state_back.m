function [out] = rescale_state_back(y, domain)
% x is the *actual* variable
% y is x scaled down to [-1,1], which is also what we give to the solver.
% Given y and the domain of x, we want to scale it back and get x.

a1 = (domain(:,2) - domain(:,1)) / 2;
t1 = (domain(:,2) + domain(:,1)) / 2;

if size(y,2) == 1
    out = diag(a1) * y + t1;
else
    out = y * diag(a1) + repmat(t1',size(y,1),1);
end