% GPOPS-II
% 
% Initial guess is not given beforehand. Number of transitions is specified
% by 'nphases'.
% 

% clear;
clc;

%-------------------------------------------------------------------------%
%--------------- Provide All Physical Data for Problem -------------------%
%-------------------------------------------------------------------------%
T = 1;
nphases = 4;
x0 = [ 0.47; 0; 0; 0.85 ];

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
clear guess bounds mesh
dt = (T) / nphases;
for iphase = 1 : nphases
%     current_phase = mod(iphase+offset,3) + 1;
%     domain = params.domain{current_phase};
%     
%     guess.phase(iphase).time = [ dt*(iphase-1); dt*iphase ];
%     guess.phase(iphase).state = [ ( domain(:,2) + domain(:,1) )' / 2;
%                                   ( domain(:,2) + domain(:,1) )' / 2 ];
%     if current_phase == 1
%         guess.phase(iphase).control = [ 0; 0 ];
%     end
%     guess.phase(iphase).integral = 0;
    
    guess.phase(iphase).time = [dt*(iphase-1); dt*iphase ];
    guess.phase(iphase).state = zeros( 2, 4 );
    guess.phase(iphase).control = [0; 0];
    guess.phase(iphase).integral = 0;
end

%-------------------------------------------------------------------------%
%----------------- Provide All Bounds for Problem ------------------------%
%-------------------------------------------------------------------------%

t0 = 0;

% Mode: 2 -> 1 -> 2 -> 1 (in spotless def)
% Phase: 1 -> 2 -> 3 -> 4 (In gpops def)

for iphase = 1 : nphases
    cmode = mod(iphase,2) + 1;      % current mode
    
    bounds.phase(iphase).initialtime.lower = t0;
    bounds.phase(iphase).initialtime.upper = t0;
    if (iphase == nphases)
        bounds.phase(iphase).finaltime.lower = T;
    else
        bounds.phase(iphase).finaltime.lower = t0;
    end
    
    bounds.phase(iphase).finaltime.upper = T;
    
    bounds.phase(iphase).initialstate.lower = x0';
    bounds.phase(iphase).initialstate.upper = x0';
    bounds.phase(iphase).state.lower        = params.domain{cmode}(:,1)';
    bounds.phase(iphase).state.upper        = params.domain{cmode}(:,2)';
    bounds.phase(iphase).finalstate.lower   = params.domain{cmode}(:,1)';
    bounds.phase(iphase).finalstate.upper   = params.domain{cmode}(:,2)';
    bounds.phase(iphase).control.lower = 0;
    bounds.phase(iphase).control.upper = params.umax;
    bounds.phase(iphase).integral.lower = -1000;
    bounds.phase(iphase).integral.upper = 1000;
    if (cmode == 1)
        % y \in [0, yR]
        bounds.phase(iphase).path.lower = 0;
        bounds.phase(iphase).path.upper = params.yR;
    else
        % y \in [yR, lmax]
        bounds.phase(iphase).path.lower = params.yR;
        bounds.phase(iphase).path.upper = params.lmax;
    end
    
end

for iphase = 1 : nphases-1
    cmode = mod(iphase,2) + 1;      % current mode
    switch cmode
        case 1              % Mode 1 -> Mode 2
            % y = yR, ydot > 0
            bounds.eventgroup(iphase).lower = [zeros(1,5), params.yR, 0];
            bounds.eventgroup(iphase).upper = [zeros(1,5), params.yR, 10 ];
            
        case 2              % Mode 1 -> Mode 2
            % y = yR, ydot < 0
            bounds.eventgroup(iphase).lower = [zeros(1,5), params.yR, -10];
            bounds.eventgroup(iphase).upper = [zeros(1,5), params.yR, 0];
    end
end

% Terminal condition
% t_final = T
iphase = nphases;
bounds.eventgroup(iphase).lower = 0;
bounds.eventgroup(iphase).upper = 0;

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method          = 'hp-LiuRao-Legendre';
mesh.maxiterations   = 45;
mesh.tolerance       = 1e-7;
for i = 1 : nphases
    mesh.phase(i).colpoints = 10 * ones(1,100);
    mesh.phase(i).fraction = 0.01 * ones(1,100);
end


%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
setup.name                           = 'SLIP_constV1_fixedT';
setup.functions.continuous           = @NatureModelContinuous;
setup.functions.endpoint             = @NatureModelEndpoint;
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

