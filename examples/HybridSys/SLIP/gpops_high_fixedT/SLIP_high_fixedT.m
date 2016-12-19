% GPOPS-II
% SLIP model, fixed final time problem. i.e., keep jumping for a certain
% amout of time
% 

% clear;
clc;

%-------------------------------------------------------------------------%
%--------------- Provide All Physical Data for Problem -------------------%
%-------------------------------------------------------------------------%
nphases = 3;
T = 1;
x0 = [ -1, 0.3, 0.2, 0 ];
offset = 1;             % Initial mode: mode 3 = 1, mode 2 = 0, mode 1 = 2

params = SLIPParams;
auxdata = struct;
auxdata.l0 = params.l0;
auxdata.yR = params.yR;
auxdata.nphases = nphases; 
auxdata.T = T;
auxdata.init = offset;


%-------------------------------------------------------------------------%
%----------------- Provide All Bounds for Problem ------------------------%
%-------------------------------------------------------------------------%

t0 = 0;
params = SLIPParams;

% Mode:  3->1->2->3->1->2->3->1->2->3->1 (In spotless def)
% Phase: 1->2->3->4->5->6->7->8->9->10->11 (In gpops def)

for iphase = 1 : nphases
    idx = mod(iphase+offset,3) + 1;
    domain = params.domain_size{idx};
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
        bounds.phase(iphase).control.lower = -1;
        bounds.phase(iphase).control.upper = 1;
    end
    bounds.phase(iphase).integral.lower = 0;
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
mesh.tolerance       = 1e-5;
for i = 1 : nphases
    mesh.phase(i).colpoints = 4 * ones(1,10);
    mesh.phase(i).fraction = 0.1 * ones(1,10);
end


%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
setup.name                           = 'SLIP_far_minimum_time';
setup.functions.continuous           = @SLIPContinuous_high_fixedT;
setup.functions.endpoint             = @SLIPEndpoint_high_fixedT;
setup.displaylevel                   = 2;
setup.bounds                         = bounds;
setup.guess                          = guess;
setup.auxdata                        = auxdata;
setup.mesh                           = mesh;
setup.nlp.solver                     = 'ipopt';
setup.nlp.snoptoptions.tolerance     = 1e-10;
setup.nlp.snoptoptions.maxiterations = 20000;
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.nlp.ipoptoptions.tolerance     = 1e-10;
% setup.derivatives.supplier           = 'adigator';
% setup.derivatives.derivativelevel    = 'second';
setup.method                         = 'RPM-Differentiation';


%-------------------------------------------------------------------------%
%---------------------- Solve Problem Using GPOPS2 -----------------------%
%-------------------------------------------------------------------------%
tic
output = gpops2(setup);
toc


