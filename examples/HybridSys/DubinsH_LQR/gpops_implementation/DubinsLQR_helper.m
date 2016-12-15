function [out] = DubinsLQR_helper(id, seq, domain, x0, guess_helper )


T = 3;
t0 = 0;
bounds = [];
guess = [];
mesh = [];
setup = [];

for iphase = 1 : length(seq)
    
    mode = seq(iphase);
    dt = (T-t0) / length(seq);
    
%-------------------------------------------------------------------------%
%----------------- Provide All Bounds for Problem ------------------------%
%-------------------------------------------------------------------------%
    if iphase == 1
        bounds.phase(iphase).initialtime.lower = t0;
        bounds.phase(iphase).initialtime.upper = t0;
        bounds.phase(iphase).initialstate.lower = x0;
        bounds.phase(iphase).initialstate.upper = x0;
    else
        bounds.phase(iphase).initialtime.lower = t0;
        bounds.phase(iphase).initialtime.upper = T;
        bounds.phase(iphase).initialstate.lower = domain{mode}(:,1)';
        bounds.phase(iphase).initialstate.upper = domain{mode}(:,2)';
    end
        
    if iphase == length(seq)
        bounds.phase(iphase).finaltime.lower = T;
        bounds.phase(iphase).finaltime.upper = T;
    else
        bounds.phase(iphase).finaltime.lower = t0;
        bounds.phase(iphase).finaltime.upper = T;
    end
        
    bounds.phase(iphase).state.lower = domain{mode}(:,1)';
    bounds.phase(iphase).state.upper = domain{mode}(:,2)';
    bounds.phase(iphase).finalstate.lower = domain{mode}(:,1)';
    bounds.phase(iphase).finalstate.upper = domain{mode}(:,2)';
    bounds.phase(iphase).control.lower = [ 0, -3 ];
    bounds.phase(iphase).control.upper = [ 1, 3 ];
    bounds.phase(iphase).integral.lower = 0;
    bounds.phase(iphase).integral.upper = 100000;
    bounds.phase(iphase).duration.lower = 1e-6;
    bounds.phase(iphase).duration.upper = T-t0;
    
    if iphase == length(seq)
        bounds.eventgroup(iphase).lower = T;
        bounds.eventgroup(iphase).upper = T;
    else
        bounds.eventgroup(iphase).lower = zeros(1,4);
        bounds.eventgroup(iphase).upper = zeros(1,4);
    end
    
%-------------------------------------------------------------------------%
%---------------------- Provide Guess of Solution ------------------------%
%-------------------------------------------------------------------------%
    
    guess.phase(iphase).time = t0 + [ (iphase-1)*dt; iphase*dt ];
    if iphase == 1
        state = x0;
    else
        state = guess_helper{seq(iphase-1), seq(iphase)};
    end

    if iphase == length(seq)
        state = [ state; (domain{mode}(:,2)+domain{mode}(:,1))' /2 ];
        guess.phase(iphase).state = state;
    else
        state = [ state; guess_helper{seq(iphase), seq(iphase+1)} ];
        guess.phase(iphase).state = state;
    end
    
    guess.phase(iphase).control = [ 0 0; 0 0 ];
    guess.phase(iphase).integral = 0;
    
end

%-------------------------------------------------------------------------%
%----------Provide Mesh Refinement Method and Initial Mesh ---------------%
%-------------------------------------------------------------------------%
mesh.method          = 'hp-LiuRao-Legendre';
mesh.tolerance       = 1e-5;
mesh.colpointsmin    = 4;
mesh.colpointsmax    = 8;
for i = 1 : length(seq)
    mesh.phase(i).colpoints = 4 * ones(1,10);
    mesh.phase(i).fraction = 0.1 * ones(1,10);
end
mesh.maxiterations   = 20;


%-------------------------------------------------------------------------%
%------------- Assemble Information into Problem Structure ---------------%        
%-------------------------------------------------------------------------%
auxdata.seq = seq;
auxdata.domain = domain;

setup.name                           = ['DubinsCar_LQR',num2str(id)];
setup.functions.continuous           = @DubinsLQRContinuous;
setup.functions.endpoint             = @DubinsLQREndpoint;
setup.displaylevel                   = 1;
setup.bounds                         = bounds;
setup.guess                          = guess;
setup.auxdata                        = auxdata;
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
out = gpops2(setup);
out.obj = out.result.objective;
