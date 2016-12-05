function [out] = Flight_f(x,~)
params = SLIPParams;
g = params.g;

% out = [ x(2);
%         0;
%         x(4);
%         -g ];

% Vectorize
out = [ x(2,:);
        0 * x(2,:);
        x(4,:);
        -g + 0 * x(2,:) ];