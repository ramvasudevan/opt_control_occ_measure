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

% The following parameters are for 3 mode system, with vertical partition
% of the flight phase, which I think makes more sense than the horizontal
% one.
yR  = params.l0 * cos(params.alpha);
% params.domain_size{1} =...
%                  [ 0.6, 1;
%                    -1.5, 1.5;
%                    -pi/4, pi/4;
%                    -3, 0;
%                    0, 2 ];
params.domain_size{2} = ...
                 [ 0, 2;
                   1.5, 2;
                   0.3, 1.2;
                   0, 1 ];
params.domain_size{3} = ...
                 [ 0, 2;
                   1.5, 2;
                   yR, 1.2;
                   -1, 0 ];


% The following parameters are for 3 mode system, but with horizontal
% partition of the flight phase.
% params.domain_size{2} = ...
%                  [ 0, 2;
%                    1.5, 2;
%                    0.3, yR;
%                    -1, 1 ];
% params.domain_size{3} = ...
%                  [ 0, 2;
%                    1.5, 2;
%                    yR, 1.2;
%                    -1, 1 ];


% The following parameters are for SimpleModel2.m only.
% To run run_SLIP.m or anything, please use the other definitions of
% domains
params.domain_size{1} = ...
                 [ 0.5, 1;
                   -5, 5;
                   -pi/3, pi/3;
                   -2*pi/3, 2*pi/3;
                   0, 5 ];

% Warning! If domain2 and domain3 are not the same, otherwise it won't be
% identity reset map between 2 and 3.
