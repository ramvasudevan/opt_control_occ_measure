% SLIP w/ 4 legs: IC optimization
% Dynamics from Z.G.
clear;
domain = [  0.7, 1.2
            0.7, 1.8
           -1.3, 1.3
%            -0.5, 0.5
%            -0.5, 0.5
           -0.5, 0.5
           -1.4, 1
           -0.5, 0.5
           -1.4, 1 ];

% domain0 = [ 1.01391108000981, 1.01391108000981      % xdot
%             1.59710245806802, 1.59710245806802      % y
%                            0,                0      % ydot
%                            0,                0      % alpha_L
%             0.89705847427842, 0.89705847427842      % alpha_L_dot
%                            0,                0      % alpha_R
%             0.89705847427834, 0.89705847427834 ];   % alpha_R_dot
domain0 = [ 1.01391108000981, 1.01391108000981      % xdot
            1.59710245806802, 1.59710245806802      % y
                           0,                0      % ydot
                           0,                0      % alpha_L
                         0.5,                1      % alpha_L_dot
                           0,                0      % alpha_R
                         0.5,                1 ];   % alpha_R_dot


%-------------------------------------------------------------------------%
%-------------------------- Parameters for OCP ---------------------------%
%-------------------------------------------------------------------------%
d = 4;              % degree of relaxation
T = 3;              % time horizon
nmodes = 4;         % number of modes

% Solver options
options.freeFinalTime = 0;      % fixed terminal time
options.withInputs = 1;         % control extraction?
options.svd_eps = 1e4;          % svd threshould for moment matrices

%-------------------------------------------------------------------------%
%---------------------------- Construct OCP ------------------------------%
%-------------------------------------------------------------------------%
t = msspoly( 't', 1 );
x = cell( nmodes, 1 );
u = cell( nmodes, 1 );
f = cell( nmodes, 1 );
g = cell( nmodes, 1 );
x0 = cell( nmodes, 1 );
hX0 = cell( nmodes, 1 );
hX = cell( nmodes, 1 );
hU = cell( nmodes, 1 );
hXT = cell( nmodes, 1 );
sX = cell( nmodes, nmodes );
R = cell( nmodes, nmodes );
h = cell( nmodes, 1 );
H = cell( nmodes, 1 );
c = cell( nmodes, 1 );

mssvar = msspoly( 'x', 7 );
uvar = msspoly( 'u', 2 );
var = [ 0; mssvar(1:3); 0; 0; mssvar(4:7) ];
x{1} = mssvar;
x{2} = mssvar;
x{3} = mssvar;
x{4} = mssvar;


% ---------------------- Dynamics -----------------------
alpha = 1000;

f{1} = T * FF_Dyn_poly_f( var );
f{2} = T * FR_Dyn_poly_f( var );
f{3} = T * LF_Dyn_poly_f( var );
f{4} = T * LR_Dyn_poly_f( var );
g{1} = T * FF_Dyn_poly_g( var );
g{2} = T * FR_Dyn_poly_g( var );
g{3} = T * LF_Dyn_poly_g( var );
g{4} = T * LR_Dyn_poly_g( var );
u{1} = uvar;
u{2} = uvar;
u{3} = uvar;
u{4} = uvar;

% -------------- Domains/guards/reset maps---------------
guard_helper1 = Guard_L_poly( var );
guard_helper2 = Guard_R_poly( var );
y1 = guard_helper1(1);
dy1 = guard_helper1(2);
y2 = guard_helper2(1);
dy2 = guard_helper2(2);


% c_helper = 10 *  ( guard_helper1(1) - guard_helper2(1) )^2;
c_helper = ( var(7) - var(9) )^2;           % (alphaB - alphaF)^2
% Mode 1 : FF
f{1} = f{1}( [2:4, 7:10] );
g{1} = g{1}( [2:4, 7:10], : );
hU{1} = uvar .* (1-uvar);
% hX{1} = ...
%     [ mssvar - domain(:,1)
%       domain(:,2) - mssvar
%       guard_helper1(1)
%       guard_helper2(1) ];
% sX{1,2} = ...       % FF -> FR
%     [ guard_helper2(1)          % y2 == 0
%       -guard_helper2(1)
%       -guard_helper2(2) ];      % dy2 <= 0
sX{1,2} = -alpha * y2^2 - dy2;      % approximate {y2=0; dy2<=0} as {-alpha * y2^2 - dy2 >= 0}

R{1,2} = Reset_R_poly( var );
R{1,2} = R{1,2}( [2:4, 7:10] );
c{1,2} = c_helper;

% sX{1,3} = ...       % FF -> LF
%     [ guard_helper1(1)          % y1 == 0
%       -guard_helper1(1)
%       -guard_helper1(2) ];      % dy1 <= 0
sX{1,3} = -alpha * y1^2 - dy1;      % approximate {y1=0; dy1<=0} as {-alpha * y1^2 - dy1 >= 0}
R{1,3} = Reset_L_poly( var );
R{1,3} = R{1,3}( [2:4, 7:10] );
c{1,3} = c_helper;

hX{1} = ...
    [ mssvar - domain(:,1)
      domain(:,2) - mssvar
      -sX{1,2}
      -sX{1,3} ];


% Mode 2 : FR
f{2} = f{2}( [2:4, 7:10] );
g{2} = g{2}( [2:4, 7:10], : );
hU{2} = uvar .* (1-uvar);
% hX{2} = ...
%     [ mssvar - domain(:,1)
%       domain(:,2) - mssvar
%       guard_helper1(1)
%       -guard_helper2(1) ];
% sX{2,1} = ...       % FR -> FF
%     [ guard_helper2(1)          % y2 == 0
%       -guard_helper2(1)
%       guard_helper2(2) ];       % dy2 >= 0
sX{2,1} = dy2 - alpha * y2^2;       % approximate {y2=0; dy2>=0} as {dy2 - alpha * y2^2 >= 0}
R{2,1} = mssvar;
c{2,1} = c_helper;

% sX{2,4} = ...       % FR -> LR
%     [ guard_helper1(1)          % y1 == 0
%       -guard_helper1(1)
%       -guard_helper1(2) ];      % dy1 <= 0
sX{2,4} = -alpha * y1^2 - dy1;      % approximate {y1=0; dy1<=0} as {-alpha * y1^2 - dy1 >= 0}
R{2,4} = Reset_L_poly( var );
R{2,4} = R{2,4}( [2:4, 7:10] );
c{2,4} = c_helper;

hX{2} = ...
    [ mssvar - domain(:,1)
      domain(:,2) - mssvar
      -sX{2,1}
      -sX{2,4} ];


% Mode 3 : LF
f{3} = f{3}( [2:4, 7:10] );
g{3} = g{3}( [2:4, 7:10], : );
hU{3} = uvar .* (1-uvar);
% hX{3} = ...
%     [ mssvar - domain(:,1)
%       domain(:,2) - mssvar
%       -guard_helper1(1)
%       guard_helper2(1) ];
% sX{3,1} = ...       % LF -> FF
%     [ guard_helper1(1)          % y1 == 0
%       -guard_helper1(1)
%       guard_helper1(2) ];       % dy1 >= 0
sX{3,1} = dy1 - alpha * y1^2;       % approximate {y1=0; dy1>=0} as {dy1 - alpha * y1^2 >= 0}
R{3,1} = mssvar;
c{3,1} = c_helper;

% sX{3,4} = ...       % LF -> LR
%     [ guard_helper2(1)          % y2 == 0
%       -guard_helper2(1)
%       -guard_helper2(2) ];      % dy2 <= 0
sX{3,4} = -alpha * y2^2 - dy2;      % approximate {y2=0; dy2<=0} as {-alpha * y2^2 - dy2 >= 0}
R{3,4} = Reset_R_poly( var );
R{3,4} = R{3,4}( [2:4, 7:10] );
c{3,4} = c_helper;

hX{3} = ...
    [ mssvar - domain(:,1)
      domain(:,2) - mssvar
      -sX{3,1}
      -sX{3,4} ];


% Mode 4 : LR
f{4} = f{4}( [2:4, 7:10] );
g{4} = g{4}( [2:4, 7:10], : );
hU{4} = uvar .* (1-uvar);
% hX{4} = ...
%     [ mssvar - domain(:,1)
%       domain(:,2) - mssvar
%       -guard_helper1(1)
%       -guard_helper2(1) ];
% sX{4,2} = ...       % LR -> FR
%     [ guard_helper1(1)          % y1 == 0
%       -guard_helper1(1)
%       guard_helper1(2) ];       % dy1 >= 0
sX{4,2} = dy1 - alpha * y1^2;       % approximate {y1=0; dy1>=0} as {dy1 - alpha * y1^2 >= 0}
R{4,2} = mssvar;
c{4,2} = c_helper;

% sX{4,3} = ...       % LR -> LF
%     [ guard_helper2(1)          % y2 == 0
%       -guard_helper2(1)
%       guard_helper2(2) ];       % dy2 >= 0
sX{4,3} = dy2 - alpha * y2^2;       % approximate {y2=0; dy2>=0} as {dy2 - alpha * y2^2 >= 0}
R{4,3} = mssvar;
c{4,3} = c_helper;

hX{4} = ...
    [ mssvar - domain(:,1)
      domain(:,2) - mssvar
      -sX{4,2}
      -sX{4,3} ];


% -------------- Cost functions ---------------
h{1} = - T * mssvar( 2 );       % Jump high
h{1} = msspoly( 0 );
h{2} = 0.5 * uvar(2)^2;
h{3} = 0.5 * uvar(1)^2;
h{4} = 0.5 * (uvar') * (uvar);

H{1} = msspoly( 0 );
H{2} = msspoly( 0 );
H{3} = msspoly( 0 );
H{4} = msspoly( 0 );

% Initial condition and Target Set
hX0{1} = [ mssvar - domain0(:,1)
           domain0(:,2) - mssvar
           guard_helper1(1)
           guard_helper2(1) ];
x0{1} = [ 1.01391108000981; 1.59710245806802; 0; 0; 0.7; 0; 0.6 ];  % xdot, y, ydot,alphaB, alphaBdot, alphaF, alphaFdot

% Target set is the entire space
hXT{1} = hX{1};
hXT{2} = hX{2};
hXT{3} = hX{3};
hXT{4} = hX{4};

%-------------------------------------------------------------------------%
%-------------------------------- Solve ----------------------------------%
%-------------------------------------------------------------------------%
% [out] = HybridOCPDualSolver_IC_noInput(t,x,u,f,g,hX,hU,sX,R,hX0,hXT,h,H,c,d,options);
[out] = HybridOCPDualSolver_switching(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,c,d,options);






