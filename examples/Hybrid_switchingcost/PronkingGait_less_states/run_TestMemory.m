% SLIP w/ 4 legs: IC optimization
% Dynamics from Z.G.
clear;
domain = [  
            0.7, 1.2
            0.7, 1.8
           -1.3, 1.3
           -0.5, 0.5
           -0.5, 0.5
           -0.5, 0.5
           -1.4, 1
           -0.5, 0.5
           -1.4, 1 ];


%-------------------------------------------------------------------------%
%-------------------------- Parameters for OCP ---------------------------%
%-------------------------------------------------------------------------%
d = 6;              % degree of relaxation
T = 4;              % time horizon
nmodes = 4;         % number of modes

% Solver options
options.freeFinalTime = 0;      % fixed terminal time
options.withInputs = 0;         % control extraction?
% options.svd_eps = 1e4;          % svd threshould for moment matrices

%-------------------------------------------------------------------------%
%---------------------------- Construct OCP ------------------------------%
%-------------------------------------------------------------------------%
t = msspoly( 't', 1 );
x = cell( nmodes, 1 );
u = cell( nmodes, 1 );
f = cell( nmodes, 1 );
g = cell( nmodes, 1 );
hX0 = cell( nmodes, 1 );
hX = cell( nmodes, 1 );
hU = cell( nmodes, 1 );
hXT = cell( nmodes, 1 );
sX = cell( nmodes, nmodes );
R = cell( nmodes, nmodes );
h = cell( nmodes, 1 );
H = cell( nmodes, 1 );
c = cell( nmodes, 1 );

mssvar = msspoly( 'x', 6 );
x{1} = mssvar;
x{2} = mssvar;
x{3} = mssvar;
x{4} = mssvar;

var_idx = [ 1:2, 6:9 ];
domain = domain(var_idx,:);
var = [ mssvar(1:2); 0; 0; 0; mssvar(3:6) ];

% ---------------------- Dynamics -----------------------
f{1} = FF_Dyn_poly( var );
f{2} = FR_Dyn_poly( var );
f{3} = LF_Dyn_poly( var );
f{4} = LR_Dyn_poly( var );

% -------------- Domains/guards/reset maps---------------
guard_helper1 = Guard_L_poly( var );
guard_helper2 = Guard_R_poly( var );
c_helper = ( guard_helper1(1) - guard_helper2(1) )^2;
% Mode 1 : FF
f{1} = f{1}( var_idx );
hX{1} = ...
    [ mssvar - domain(:,1)
      domain(:,2) - mssvar
      guard_helper1(1)
      guard_helper2(1) ];
sX{1,2} = ...       % FF -> FR
    [ guard_helper2(1)          % y2 == 0
      -guard_helper2(1)
      -guard_helper2(2) ];      % dy2 <= 0
R{1,2} = Reset_R_poly( var );
R{1,2} = R{1,2}( var_idx );
c{1,2} = c_helper;

sX{1,3} = ...       % FF -> LF
    [ guard_helper1(1)          % y1 == 0
      -guard_helper1(1)
      -guard_helper1(2) ];      % dy1 <= 0
R{1,3} = Reset_L_poly( var );
R{1,3} = R{1,3}( var_idx );
c{1,3} = c_helper;

% Mode 2 : FR
f{2} = f{2}( var_idx );
hX{2} = ...
    [ mssvar - domain(:,1)
      domain(:,2) - mssvar
      guard_helper1(1)
      -guard_helper2(1) ];
sX{2,1} = ...       % FR -> FF
    [ guard_helper2(1)          % y2 == 0
      -guard_helper2(1)
      guard_helper2(2) ];       % dy2 >= 0
R{2,1} = mssvar;
c{2,1} = c_helper;

sX{2,4} = ...       % FR -> LR
    [ guard_helper1(1)          % y1 == 0
      -guard_helper1(1)
      -guard_helper1(2) ];      % dy1 <= 0
R{2,4} = Reset_L_poly( var );
R{2,4} = R{2,4}( var_idx );
c{2,4} = c_helper;

% Mode 3 : LF
f{3} = f{3}( var_idx );
hX{3} = ...
    [ mssvar - domain(:,1)
      domain(:,2) - mssvar
      -guard_helper1(1)
      guard_helper2(1) ];
sX{3,1} = ...       % LF -> FF
    [ guard_helper1(1)          % y1 == 0
      -guard_helper1(1)
      guard_helper1(2) ];       % dy1 >= 0
R{3,1} = mssvar;
c{3,1} = c_helper;

sX{3,4} = ...       % LF -> LR
    [ guard_helper2(1)          % y2 == 0
      -guard_helper2(1)
      -guard_helper2(2) ];      % dy2 <= 0
R{3,4} = Reset_R_poly( var );
R{3,4} = R{3,4}( var_idx );
c{3,4} = c_helper;

% Mode 4 : LR
f{4} = f{4}( var_idx );
hX{4} = ...
    [ mssvar - domain(:,1)
      domain(:,2) - mssvar
      -guard_helper1(1)
      -guard_helper2(1) ];
sX{4,2} = ...       % LR -> FR
    [ guard_helper1(1)          % y1 == 0
      -guard_helper1(1)
      guard_helper1(2) ];       % dy1 >= 0
R{4,2} = mssvar;
c{4,2} = c_helper;

sX{4,3} = ...       % LR -> LF
    [ guard_helper2(1)          % y2 == 0
      -guard_helper2(1)
      guard_helper2(2) ];       % dy2 >= 0
R{4,3} = mssvar;
c{4,3} = c_helper;

% -------------- Cost functions ---------------
h{1} = msspoly( 0 );
h{2} = msspoly( 0 );
h{3} = msspoly( 0 );
h{4} = msspoly( 0 );

H{1} = msspoly( 0 );
H{2} = msspoly( 0 );
H{3} = msspoly( 0 );
H{4} = msspoly( 0 );

% Initial condition and Target Set
hX0{1} = [ hX{1}
           mssvar(1) - 1
           1 - mssvar(1) ];

% Target set is the entire space
hXT{1} = hX{1};
hXT{2} = hX{2};
hXT{3} = hX{3};
hXT{4} = hX{4};

%-------------------------------------------------------------------------%
%-------------------------------- Solve ----------------------------------%
%-------------------------------------------------------------------------%
[out] = HybridOCPDualSolver_IC_noInput(t,x,u,f,g,hX,hU,sX,R,hX0,hXT,h,H,c,d,options);






