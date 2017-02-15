% Dubins car model, minimum time problem, with a shortcut mode 2.
% Given transition sequence: 1-3 (i.e., mode 2 is not visited)
% -------
% |  1  |    |
% |-----|    | <- 2
% |  3  |    |
% -------
% 
% state = (x,y,theta)
% control = (V,omega)
% 

clear;
clc;

%-------------------------------------------------------------------------%
%----------------- Provide All Bounds for Problem ------------------------%
%-------------------------------------------------------------------------%
T = 3;
t0 = 0;
x0 = [ -0.8, 0.8, 0 ];

iphase = 1;
bounds.phase(iphase).initialtime.lower = t0;
bounds.phase(iphase).initialtime.upper = t0;
bounds.phase(iphase).finaltime.lower = t0;
bounds.phase(iphase).finaltime.upper = T;
bounds.phase(iphase).initialstate.lower = x0;
bounds.phase(iphase).initialstate.upper = x0;
bounds.phase(iphase).state.lower = [ -1, 0, -pi/2 ];
bounds.phase(iphase).state.upper = [ 1, 1, pi/2 ];
bounds.phase(iphase).finalstate.lower = [ -1, 0, -pi/2 ];
bounds.phase(iphase).finalstate.upper = [ 1, 1, pi/2 ];
bounds.phase(iphase).control.lower = [ 0, -3 ];
bounds.phase(iphase).control.upper = [ 1, 3 ];
bounds.phase(iphase).integral.lower = 0;
bounds.phase(iphase).integral.upper = 100000;

iphase = 2;
bounds.phase(iphase).initialtime.lower = t0;
bounds.phase(iphase).initialtime.upper = T;
bounds.phase(iphase).finaltime.lower = t0;
bounds.phase(iphase).finaltime.upper = T;
bounds.phase(iphase).initialstate.lower = [ -1, -1, -pi/2 ];
bounds.phase(iphase).initialstate.upper = [ 1, 0, pi/2 ];
bounds.phase(iphase).state.lower = [ -1, -1, -pi/2 ];
bounds.phase(iphase).state.upper = [ 1, 0, pi/2 ];
bounds.phase(iphase).finalstate.lower = [ -1, -1, -pi/2 ];
bounds.phase(iphase).finalstate.upper = [ 1, 0, pi/2 ];
bounds.phase(iphase).control.lower = [ 0, -3 ];
bounds.phase(iphase).control.upper = [ 1, 3 ];
bounds.phase(iphase).integral.lower = 0;
bounds.phase(iphase).integral.upper = 100000;


bounds.eventgroup(1).lower = zeros(1,4);
bounds.eventgroup(1).upper = zeros(1,4);
bounds.eventgroup(2).lower = zeros(1,2);
bounds.eventgroup(2).upper = zeros(1,2);

%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%
dt = T / 2;
t1 = dt;
t2 = 2 * dt;
% phase 1
guess.phase(1).time     = [t0; t1]; 
guess.phase(1).state    = [ x0;
                            0, 1, pi/4 ];
guess.phase(1).control  = [0, 0; 0, 0];
guess.phase(1).integral = 0;
% phase 2
guess.phase(2).time     = [t1; t2];
guess.phase(2).state    = [ 0.6, -0.8, 0;
                            0.8, -0.8, 0 ];
guess.phase(2).control  = [0, 0; 0, 0];
guess.phase(2).integral = 0;

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method          = 'hp-LiuRao-Legendre';
mesh.tolerance       = 1e-7;
mesh.colpointsmin    = 4;
mesh.colpointsmax    = 8;
for i = 1 : 2
    mesh.phase(i).colpoints = 4 * ones(1,10);
    mesh.phase(i).fraction = 0.1 * ones(1,10);
end
mesh.maxiterations   = 10;

%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
setup.name                           = 'DubinsCar_LQR';
setup.functions.continuous           = @ShortcutContinuous2;
setup.functions.endpoint             = @ShortcutEndpoint2;
setup.displaylevel                   = 1;
setup.bounds                         = bounds;
setup.guess                          = guess;
setup.mesh                           = mesh;
setup.nlp.solver                     = 'ipopt';
setup.nlp.snoptoptions.tolerance     = 1e-10;
setup.nlp.snoptoptions.maxiterations = 20000;
setup.nlp.ipoptoptions.linear_solver = 'ma57';
setup.nlp.ipoptoptions.tolerance     = 1e-10;
setup.mesh.colpointsmin              = 10;
setup.method                         = 'RPM-Differentiation';

%-------------------------------------------------------------------------%
%---------------------- Solve Problem Using GPOPS2 -----------------------%
%-------------------------------------------------------------------------%
tic
output = gpops2(setup);
toc

