function params = SLIPParams()

params = struct();

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

% domain
params.umax = 0.2;

%% 3 mode system, vertical partition of flight phase (preferred)
% x_dot > 0, for running case only.
yR  = params.l0 * cos(params.alpha);
params.yR = yR;
params.domain_size{1} =...
                 [ 0.6, 1;
                   -1.5, 1.5;
                   -pi/4, pi/4;
                   -3, 0;
                   0, 10 ];
params.domain_size{2} = ...
                 [ 0, 10;
                   1.5, 2;
                   0.3, 1.2;
                   0, 1 ];
params.domain_size{3} = ...
                 [ 0, 10;
                   1.5, 2;
                   yR, 1.2;
                   -1, 0 ];

%% 3 mode system, vertical partition of flight phase (preferred)
% % x_dot >=< 0, alpha = 0, for jumping case.
% params.alpha = 0;
% yR  = params.l0 * cos(params.alpha);
% params.domain_size{1} =...
%                  [ 0.5, 1;
%                    -1.5, 1.5;
%                    -pi/4, pi/4;
%                    -3, 3;
%                    0, 2 ];
% params.domain_size{2} = ...
%                  [ 0, 2;
%                    -1.5, 1.5;
%                    0.3, 1.5;
%                    0, 1 ];
% params.domain_size{3} = ...
%                  [ 0, 2;
%                    -1.5, 1.5;
%                    yR, 1.5;
%                    -1, 0 ];

%% 3 mode system, horizontal partition of the flight phase (less preferred)
% yR  = params.l0 * cos(params.alpha);
% params.domain_size{1} =...
%                  [ 0.6, 1;
%                    -1.5, 1.5;
%                    -pi/4, pi/4;
%                    -3, 0;
%                    0, 10 ];
% params.domain_size{2} = ...
%                  [ 0, 10;
%                    1.5, 2;
%                    0.3, yR;
%                    0, 1 ];
% params.domain_size{3} = ...
%                  [ 0, 10;
%                    1.5, 2;
%                    yR, 1.2;
%                    -1, 1 ];

%% The following parameters are for SimpleModel2.m only.
% To run run_SLIP.m or anything, please use the other definitions of
% domains
% params.domain_size{1} = ...
%                  [ 0.5, 1;
%                    -5, 5;
%                    -pi/3, pi/3;
%                    -2*pi/3, 2*pi/3;
%                    0, 5 ];

% Warning! If domain2 and domain3 are not the same, it won't be
% identity reset map between 2 and 3.
