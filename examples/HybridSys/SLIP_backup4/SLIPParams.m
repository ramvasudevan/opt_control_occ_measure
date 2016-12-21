function params = SLIPParams()

params = struct();

% dynamics
% params.m = 10;
% params.k = 1962;
% params.l0 = 1;
% params.g = 9.8;
params.m = 1;
params.k = 6;
params.g = 0.2;
params.l0 = 0.2;

% reset
params.alpha = pi/6;        % angle after reset

% domain
params.umax = 0.1;

%% 3 mode system, vertical partition of flight phase (preferred)
% x_dot > 0, for running case only.
yR  = params.l0 * cos(params.alpha);
params.yR = yR;

% params.domain_size{1} =...
%                  [ 0.1, 0.2;
%                    -0.3, 0.3;
%                    -1, 1;
%                    -3, 0;
%                    -1, 1 ];
% params.domain_size{2} = ...
%                  [ -1, 1;
%                    0, 0.5;
%                    0.15, 0.25;
%                    0, 0.2 ];
% params.domain_size{3} = ...
%                  [ -1, 1;
%                    0, 0.5;
%                    yR, 0.25;
%                    -0.2, 0 ];

params.domain_size{1} =...
                 [ 0.1, 0.2;
                   -0.3, 0.3;
                   -1, 1;
                   -3, 0;
                   -1, 1 ];
params.domain_size{2} = ...
                 [ -1, 1;
                   0, 0.5;
                   0.15, 0.5;
                   0, 1 ];
params.domain_size{3} = ...
                 [ -1, 1;
                   0, 0.5;
                   yR, 0.5;
                   -1, 0 ];
