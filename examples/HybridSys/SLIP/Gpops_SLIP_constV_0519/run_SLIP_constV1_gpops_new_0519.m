% GPOPS-II
% SLIP model with 3 modes.
% Goal: Follow a constant-speed trajectory during t \in [0,T]
% running cost = ((0.25*t/T - 1) - x)^2
% terminal cost = 0
% 
% Initial guess is not given beforehand. Number of transitions is specified
% by 'nphases'.
% 

clear;
clc;

%-------------------------------------------------------------------------%
%-------------------- All Physical Parameters for SLIP -------------------%
%-------------------------------------------------------------------------%
params = struct();

params.m        = 1;            % mass
params.k        = 6;            % spring constant
params.g        = 0.2;          % gravitational acceleration
params.l0       = 0.2;          % maximum leg length

params.alpha    = pi/6;         % leg angle in flight phase
params.umax     = 0.1;          % upper bound of input

%-------------------------------------------------------------------------%
%-------------- Domains (Defined by Upper and Lower Bounds) --------------%
%-------------------------------------------------------------------------%
yR  = params.l0 * cos(params.alpha);        % touch-down height
params.yR = yR;

% Mode 1: stance
% state = ( l, l_dot, theta, theta_dot, x )
params.domain{1} =...
        [ 0.1, 0.2;         % l         - leg length
         -0.3, 0.3;         % l_dot     - time derivative of l
           -1, 1;           % theta     - leg angle
           -3, 0;           % theta_dot - time derivative of theta
           -1, 1 ];         % x         - horizontal displacement

% Mode 2: flight, vertical velocity is positive (upwards)
% state = ( x, x_dot, y, y_dot )
params.domain{2} = ...
         [ -1, 1;           % x         - horizontal displacement
            0, 0.5;         % x_dot     - time derivative of x
         0.15, 0.5;         % y         - vertical displacement
            0, 1 ];         % y_dot     - time derivative of y

% Mode 3: flight, vertical velocity is negative (downwards)
% state = ( x, x_dot, y, y_dot )
params.domain{3} = ...
         [ -1, 1;           % x         - horizontal displacement
            0, 0.5;         % x_dot     - time derivative of x
           yR, 0.5;         % y         - vertical displacment
           -1, 0 ];         % y_dot     - time derivative of y


%-------------------------------------------------------------------------%
%--------------- Provide All Physical Data for Problem -------------------%
%-------------------------------------------------------------------------%
T = 3;
nphases = 5;
x0 = [ -0.5, 0.3, 0.2, 0 ];
offset = 1;             % Initial mode: mode 3 = 1, mode 2 = 0, mode 1 = 2

auxdata = struct;
auxdata.params = params;
auxdata.l0 = params.l0;
auxdata.yR = params.yR;
auxdata.nphases = nphases; 
auxdata.T = T;
auxdata.init = offset;

%-------------------------------------------------------------------------%
%----------------- Generate Initial Guess for Problem --------------------%
%-------------------------------------------------------------------------%
clear guess bounds mesh
dt = (T/1.5) / nphases;
for iphase = 1 : nphases
    current_phase = mod(iphase+offset,3) + 1;
    domain = params.domain{current_phase};
    
    guess.phase(iphase).time = [ dt*(iphase-1); dt*iphase ];
    guess.phase(iphase).state = [ ( domain(:,2) + domain(:,1) )' / 2;
                                  ( domain(:,2) + domain(:,1) )' / 2 ];
    if current_phase == 1
        guess.phase(iphase).control = [ 0; 0 ];
    end
    guess.phase(iphase).integral = 0;
end

%-------------------------------------------------------------------------%
%----------------- Provide All Bounds for Problem ------------------------%
%-------------------------------------------------------------------------%

t0 = 0;

% Mode:  3->1->2->3->1->2->3->1->2->3->1 (In spotless def)
% Phase: 1->2->3->4->5->6->7->8->9->10->11 (In gpops def)

for iphase = 1 : nphases
    idx = mod(iphase+offset,3) + 1;
    domain = params.domain{idx};
    bounds.phase(iphase).initialtime.lower = t0;
    bounds.phase(iphase).finaltime.upper = T;
    if iphase == 1
        bounds.phase(iphase).initialtime.upper = t0;
    else
        bounds.phase(iphase).initialtime.upper = T; 
    end
    if iphase == nphases
        bounds.phase(iphase).finaltime.lower = T;
    else
        bounds.phase(iphase).finaltime.lower = t0;
    end
    if iphase == 1
        bounds.phase(iphase).initialstate.lower = x0;
        bounds.phase(iphase).initialstate.upper = x0;
    else
        bounds.phase(iphase).initialstate.lower = domain(:,1)';
        bounds.phase(iphase).initialstate.upper = domain(:,2)';
    end
    bounds.phase(iphase).state.lower = domain(:,1)';
    bounds.phase(iphase).state.upper = domain(:,2)';
    bounds.phase(iphase).finalstate.lower = domain(:,1)';
    bounds.phase(iphase).finalstate.upper = domain(:,2)';
    if idx == 1
        bounds.phase(iphase).control.lower = 0;
        bounds.phase(iphase).control.upper = 1;
    end
    bounds.phase(iphase).integral.lower = -100000;
    bounds.phase(iphase).integral.upper = 100000;
    
end

for iphase = 1 : nphases-1
    idx = mod(iphase+offset,3) + 1;
    switch idx
        case 1              % Stance -> Flight 1 (Take-off)
            bounds.eventgroup(iphase).lower = zeros(1,7);
            bounds.eventgroup(iphase).upper = [ zeros(1,6), 1000 ];     % 1000 is supposed to be ldotmax!!!
            
        case 2              % Flight 1 -> Flight 2
            bounds.eventgroup(iphase).lower = zeros(1,6);
            bounds.eventgroup(iphase).upper = zeros(1,6);
            
        case 3              % Flight 2 -> Stance (Touch-down)
            bounds.eventgroup(iphase).lower = zeros(1,7);
            bounds.eventgroup(iphase).upper = zeros(1,7);
            
    end
end

% Terminal condition
iphase = nphases;
bounds.eventgroup(iphase).lower = 0;
bounds.eventgroup(iphase).upper = 1000;

%-------------------------------------------------------------------------%
%------- Initial Guess of Solution Should be Provided by run_sim ---------%
%-------------------------------------------------------------------------%


%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method          = 'hp-LiuRao-Legendre';
% mesh.maxiterations   = 45;
mesh.tolerance       = 1e-7;
for i = 1 : nphases
    mesh.phase(i).colpoints = 10 * ones(1,100);
    mesh.phase(i).fraction = 0.01 * ones(1,100);
end


%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
setup.name                           = 'SLIP_constV1_fixedT';
setup.functions.continuous           = @SLIPContinuous_constV1_fixedT;
setup.functions.endpoint             = @SLIPEndpoint_constV1_fixedT;
setup.displaylevel                   = 2;
setup.bounds                         = bounds;
setup.guess                          = guess;
setup.auxdata                        = auxdata;
setup.mesh                           = mesh;
setup.nlp.solver                     = 'ipopt';
setup.nlp.ipoptoptions.maxiterations = 200;
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.nlp.ipoptoptions.tolerance     = 1e-10;
% setup.derivatives.supplier           = 'adigator';
setup.derivatives.derivativelevel    = 'second';
setup.method                         = 'RPM-Differentiation';
% setup.scales.method                  = 'automatic-hybridUpdate';


%-------------------------------------------------------------------------%
%---------------------- Solve Problem Using GPOPS2 -----------------------%
%-------------------------------------------------------------------------%
tic
output = gpops2(setup);
toc

%-------------------------------------------------------------------------%
%-------------------------------- PLot -----------------------------------%
%-------------------------------------------------------------------------%
PlotGpopsTrajectory;

disp( output.result.objective );