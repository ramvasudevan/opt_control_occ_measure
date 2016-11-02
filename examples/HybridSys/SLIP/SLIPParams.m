function params = SLIPParams()

params = struct();

polysin = @(x) x - x^3/6;
polycos = @(x) 1 - x^2 + x^4/24;
% polysin = @(x) x;
% polycos = @(x) 1 - x^2;

% dynamics
% params.m = 10;
% params.k = 1962;
% params.l0 = 1;
% params.g = 9.8;
params.m = 1;
params.k = 10;
params.g = 1;
params.l0 = 1;

% reset
params.alpha = pi/6;        % angle after reset
% params.ymin = 0.5 * params.l0;
% % params.yR = params.l0 * polycos( params.alpha );
% params.yR = params.l0 * cos(params.alpha);

% domain
params.umax = 0.2;
% params.ldotlim = 5;
% params.lmaxoff = 0.5;
% params.tlim = pi/3;
% params.tdotlim = 2 * pi/3;
% params.xlim = 5;
% params.xdotlim = params.l0 * params.tdotlim;