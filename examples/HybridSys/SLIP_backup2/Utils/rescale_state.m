function [out] = rescale_state(x, domain)
% x is the *actual* variable
% y is x scaled down to [-1,1], which is also what we give to the solver
% Given x and the domain of x, we want to scale it and get what we call 'y'.

a1 = (domain(:,2) - domain(:,1)) / 2;
t1 = (domain(:,2) + domain(:,1)) / 2;

if size(x,2) == 1
    out = (x-t1) .* (1 ./ a1);
else
    out = ( x - repmat(t1', size(x,1), 1) ) * diag(1 ./ a1);
end