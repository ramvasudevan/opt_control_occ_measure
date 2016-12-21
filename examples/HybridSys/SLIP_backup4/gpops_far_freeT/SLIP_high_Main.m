% GPOPS-II
% SLIP model, get to x>=8 in minimum time.
% c.f. run_SLIP_far.m
% 

% clear;
clc;

%-------------------------------------------------------------------------%
%--------------- Provide All Physical Data for Problem -------------------%
%-------------------------------------------------------------------------%
nphases = 4;

Target = 0;

params = SLIPParams;
auxdata = struct;
auxdata.l0 = params.l0;
auxdata.yR = params.yR;
auxdata.nphases = nphases;
auxdata.Target = Target;
auxdata.init = 1;       % Initial mode, mode 3 = 1, mode 2 = 0, mode 1 = 2

%-------------------------------------------------------------------------%
%----------------- Provide All Bounds for Problem ------------------------%
%-------------------------------------------------------------------------%
T = 4;
t0 = 0;
% x0 = [ -1, 0.3, 0.2, 0 ];
x0 = [ -1, 0.3, 0.2, 0 ];
params = SLIPParams;

offset = auxdata.init;

% Mode:  3->1->2->3->1->2->3->1->2->3->1 (In spotless def)
% Phase: 1->2->3->4->5->6->7->8->9->10->11 (In gpops def)

for iphase = 1 : nphases
    idx = mod(iphase+offset,3) + 1;
    domain = params.domain_size{idx};
    bounds.phase(iphase).initialtime.lower = t0;
    if iphase == 1
        bounds.phase(iphase).initialtime.upper = t0;
    else
        bounds.phase(iphase).initialtime.upper = T; 
    end
    bounds.phase(iphase).finaltime.lower = t0;
    bounds.phase(iphase).finaltime.upper = T;
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
%     bounds.phase(iphase).duration.lower = 1e-2;
%     bounds.phase(iphase).duration.upper = 1000;
end

% bounds.phase(nphases).finaltime.lower = T;
% bounds.phase(nphases).finaltime.upper = T;

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
% bounds.eventgroup(iphase).lower = 0;
bounds.eventgroup(iphase).lower = 0;
bounds.eventgroup(iphase).upper = 0;

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%
% 
% guess.phase(1).time = [ 0; 0.5176 ];
% guess.phase(1).integral = 0.5176;
% guess.phase(1).state = [ -1, 0.3, 0.2, 0;
%                          -0.8447, 0.3000, 0.1732, -0.1035 ];
% 
% guess.phase(2).time = [ 0.5176; 1.2668 ];
% guess.phase(2).integral = 0.6438;
% guess.phase(2).state = [ 0.2000   -0.2397    0.5236   -1.0402   -0.8447;
%                          0.2000    0.3037   -0.7070   -1.0803   -0.6152 ];
% 
% guess.phase(2).control = [ 0; 0 ];
% 
% guess.phase(3).time = [ 1.2668; 1.7309 ];
% guess.phase(3).integral = 0.5;
% guess.phase(3).state = [ -0.6152    0.3674    0.1549    0.0928;
%                          -0.4446    0.3674    0.1764   -0.0000 ];
% 
% 
% guess.phase(4).time = [ 1.7309; 1.9100 ];
% guess.phase(4).integral = 0.5;
% guess.phase(4).state = [ -0.4446    0.3674    0.1764   -0.0000;
%                          -0.3788    0.3674    0.1732   -0.0358 ];
% 
% guess.phase(5).time = [ 1.9100; 2.4611 ];
% guess.phase(5).integral = 0.5;
% guess.phase(5).state = [ 0.2000   -0.2147    0.5236   -1.5013   -0.3788;
%                          0.2000    0.3338   -0.5603   -1.5038   -0.1727 ];
% guess.phase(5).control = [ 0; 0 ];
% 
% guess.phase(6).time = [ 2.4611; 2.8571 ];
% guess.phase(6).integral = 0.5;
% guess.phase(6).state = [ -0.1727    0.4325    0.1695    0.1230;
%                          -0.0014    0.4325    0.2026    0.0438 ];
% guess.phase(4).time = [ 1.2164; 1.5 ];
% guess.phase(4).integral = 0;
% guess.phase(4).state = [ 2.5, 1.7330, 1, 0;
%                          3,   1.7330, 0.5, -0.4 ];


% dt = T / (nphases-1);

% for iphase = 1 : nphases
%     idx = mod(iphase+1,3) + 1;
%     guess.phase(iphase).time        = [ t0+(iphase-1)*dt; t0+iphase*dt ];
%     guess.phase(iphase).integral    = 0;
%     switch idx
%         case 1      % Stance phase
%             guess.phase(iphase).state = [ 0.7, 0, 0, -1, (iphase-1)/2;
%                                           0.7, 0, 0, -1, (iphase)/2 ];
%             guess.phase(iphase).control = [ 0; 0 ];
%         case 2      % Flight 1
%             guess.phase(iphase).state = [ (iphase-1)/2, 1.7, 0.8, 0.5;
%                                           (iphase)/2,   1.7, 0.8, 0.5 ];
%         case 3      % Flight 2
%             guess.phase(iphase).state = [ (iphase-1)/2, 1.7, 1, -0.5;
%                                           9,            1.7, 1, -0.5 ];
%     end
% end


%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method          = 'hp-LiuRao-Legendre';
mesh.tolerance       = 1e-4;
for i = 1 : nphases
    mesh.phase(i).colpoints = 4 * ones(1,10);
    mesh.phase(i).fraction = 0.1 * ones(1,10);
end


%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
setup.name                           = 'SLIP_high';
setup.functions.continuous           = @SLIPContinuous;
setup.functions.endpoint             = @SLIPEndpoint;
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


