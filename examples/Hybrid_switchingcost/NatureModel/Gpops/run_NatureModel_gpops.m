% GPOPS-II
% 
% Initial guess is not given beforehand. Number of transitions is specified
% by 'nphases'.
% 

% clear;
% clc;

%-------------------------------------------------------------------------%
%--------------- Provide All Physical Data for Problem -------------------%
%-------------------------------------------------------------------------%
T = 0.8;
MaxTime = 1;
nphases = 1;
x0 = [ 0.47; 0; 0; 0.85 ];
% x0 = [ 0.47; 0; 0; 0 ];

auxdata = struct;
auxdata.params = params;
auxdata.l0 = params.l0;
auxdata.yR = params.yR;
auxdata.nphases = nphases; 
auxdata.T = T;
auxdata.d_des = 0.6;        % desired step length

%-------------------------------------------------------------------------%
%----------------- Generate Initial Guess for Problem --------------------%
%-------------------------------------------------------------------------%
clear guess bounds mesh setup
dt = (MaxTime) / nphases;
for iphase = 1 : nphases
    
    guess.phase(iphase).time = [dt*(iphase-1); dt*iphase ];
    guess.phase(iphase).state = zeros( 2, 4 );
    guess.phase(iphase).control = [0; 0];
    guess.phase(iphase).integral = 0;
end

guess = myguess;

%-------------------------------------------------------------------------%
%----------------- Provide All Bounds for Problem ------------------------%
%-------------------------------------------------------------------------%

t0 = 0;
MaxTime = 1;

% Mode: 2 -> 1 -> 2 -> 1 (in spotless def)
% Phase: 1 -> 2 -> 3 -> 4 (In gpops def)

for iphase = 1 : nphases
    cmode = mod(iphase,2) + 1;      % current mode
    
    bounds.phase(iphase).initialtime.lower = t0;
    bounds.phase(iphase).initialtime.upper = MaxTime;
    bounds.phase(iphase).finaltime.lower = t0;
    bounds.phase(iphase).finaltime.upper = MaxTime;
    bounds.phase(iphase).initialstate.lower = params.domain{cmode}(:,1)';
    bounds.phase(iphase).initialstate.upper = params.domain{cmode}(:,2)';
    bounds.phase(iphase).state.lower        = params.domain{cmode}(:,1)';
    bounds.phase(iphase).state.upper        = params.domain{cmode}(:,2)';
    bounds.phase(iphase).finalstate.lower   = params.domain{cmode}(:,1)';
    bounds.phase(iphase).finalstate.upper   = params.domain{cmode}(:,2)';
    
    
    if (iphase == 1)
        bounds.phase(iphase).initialtime.upper = t0;
        bounds.phase(iphase).initialstate.lower = x0';
        bounds.phase(iphase).initialstate.upper = x0';
    end
    if (iphase == nphases)
        bounds.phase(iphase).finaltime.lower = MaxTime;
    end
    bounds.phase(iphase).control.lower = 0;
    bounds.phase(iphase).control.upper = params.umax;
    bounds.phase(iphase).integral.lower = -1000;
    bounds.phase(iphase).integral.upper = 1000;
%     if (cmode == 1)
%         % y \in [0, yR]
%         bounds.phase(iphase).path.lower = 0;
%         bounds.phase(iphase).path.upper = params.yR;
%     else
%         % y \in [yR, lmax]
%         bounds.phase(iphase).path.lower = params.yR;
%         bounds.phase(iphase).path.upper = params.lmax;
%     end
    
%     bounds.phase(iphase).duration.lower = 0.1;
%     bounds.phase(iphase).duration.upper = MaxTime;
    
end

for iphase = 1 : nphases-1
    cmode = mod(iphase,2) + 1;      % current mode
    switch cmode
        case 1              % Mode 1 -> Mode 2
            % y = yR, ydot > 0
            bounds.eventgroup(iphase).lower = [zeros(1,5), params.yR, 0.01];
            bounds.eventgroup(iphase).upper = [zeros(1,5), params.yR, 10 ];
            
        case 2              % Mode 1 -> Mode 2
            % y = yR, ydot < 0
            bounds.eventgroup(iphase).lower = [zeros(1,5), params.yR, -10];
            bounds.eventgroup(iphase).upper = [zeros(2.21,5), params.yR, 0];
    end
end

% Terminal condition
% t_final = T
iphase = nphases;
bounds.eventgroup(iphase).lower = MaxTime;
bounds.eventgroup(iphase).upper = MaxTime;

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method          = 'hp-LiuRao-Legendre';
mesh.maxiterations   = 10;
mesh.tolerance       = 1e-6;
mesh.colpointsmin    = 2;
mesh.colpointsmax    = 14;
for i = 1 : nphases
    mesh.phase(i).colpoints = 4 * ones(1,10);
    mesh.phase(i).fraction = 0.1 * ones(1,10);
end


%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
setup.name                           = 'NatureModel_minimize_u2';
setup.functions.continuous           = @NatureModelContinuous;
setup.functions.endpoint             = @NatureModelEndpoint;
setup.displaylevel                   = 2;
setup.bounds                         = bounds;
setup.guess                          = guess;
setup.auxdata                        = auxdata;
setup.mesh                           = mesh;
setup.nlp.solver                     = 'ipopt';
setup.nlp.snoptoptions.tolerance     = 1e-6;2.2
setup.nlp.snoptoptions.maxiterations = 20;
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.nlp.ipoptoptions.tolerance     = 1e-6;
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
%%
state_hist_gpops = [];
t_hist_gpops = [];
control_hist_gpops = [];
for iphase = 1 : nphases
    t_hist_gpops = [ t_hist_gpops; output.result.solution.phase(iphase).time ];
    state_hist_gpops = [ state_hist_gpops; output.result.solution.phase(iphase).state ];
    control_hist_gpops = [ control_hist_gpops; output.result.solution.phase(iphase).control ];
end
l_hist = state_hist_gpops( :, 1 );
ldot_hist = state_hist_gpops( :, 2 );
theta_hist = state_hist_gpops( :, 3 );
thetadot_hist = state_hist_gpops( :, 4 );
x_hist = l_hist .* polysin( theta_hist );
y_hist = l_hist .* polycos( theta_hist );
figure(7);
hold on;
title('states');
plot(t_hist_gpops, l_hist);
plot(t_hist_gpops, ldot_hist);
plot(t_hist_gpops, theta_hist);
plot(t_hist_gpops, thetadot_hist);
plot(t_hist_gpops, control_hist_gpops);

legend('l','ldot','theta','thetadot','u');

% plot(t_hist, control_hist);
figure(8);
plot(x_hist, y_hist);
title('X-Y');