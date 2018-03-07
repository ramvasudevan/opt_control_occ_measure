% Shortcut problem with 3 modes: minimum time problem
% 
% Mode 1 and mode 2: Dubins car
% State variables: [x, y, theta]'   in [-1,1]x[-1,1]x[-pi/2,pi/2]
% Input: [ V, omega ]'              in [0,1]x[-1,1]
% System dynamics:
% xdot = [ V*cos(theta)
%          V*sin(theta)
%          3 * omega ]
% 
% Mode 3: shortcut path
% State variables: x                in [-1,1]
% Input: V                          in [0,1]
% xdot = -2 * V
% 
% Hybrid modes definition:
% -------
% |  1  | -> |
% |-----|    | 3
% |  2  | <- |
% -------
% 
% Trajectory starts at (-0.8,0.8,0) in mode 1, and arrives at (0.8,-0.8)
% in mode 2, in minimum time.
% 
% Comments:
%   * Dimensions are different in each mode
%   * The transition sequences 1->3->2 and 1->2 are both feasible, but the
% sequence 1->3->2 is prefered because we are allowed to move 'faster' in
% mode 3 (i.e., mode 3 is a shortcut).
%   * R_13 = 1 is not injective, in fact the Jacobian is zero.
% 

clear;
T = 3;          % maximum time horizon
d = 6;          % degree of relaxation
dr = 8;
nmodes = 3;     % number of hybrid modes

polysin = @(x) x;
polycos = @(x) 1 - x^2/2;

% ========================= Define variables =============================
t = msspoly( 't', 1 );
x = cell( nmodes, 1 );
u = cell( nmodes, 1 );
f = cell( nmodes, 1 );
g = cell( nmodes, 1 );
x0 = cell( nmodes, 1 );
hX = cell( nmodes, 1 );
hU = cell( nmodes, 1 );
hXT = cell( nmodes, 1 );
sX = cell( nmodes, nmodes );
R = cell( nmodes, nmodes );
h = cell( nmodes, 1 );
H = cell( nmodes, 1 );

x0{1} = [ -0.8; 0.8; 0 ];

% =============================== Dynamics ===============================
% -------  Mode 1 --------
x{1} = msspoly( 'xa', 3 );
ua = msspoly( 'ua', 2 );
u{1} = ua;
y = x{1};
f{1} = T * [ 0;
             0;
             0 ];
g{1} = T * [ polycos(y(3)), 0;
             polysin(y(3)), 0;
             0,             3 ];
% -------  Mode 2 --------
x{2} = x{1};
u{2} = u{1};
f{2} = f{1};
g{2} = g{1};
               
% -------  Mode 3 --------
x{3} = msspoly( 'xb', 1 );
ub = msspoly( 'ub', 1 );
u{3} = ub;
f{3} = T * 0;
g{3} = T * [ -2 ];

% ======================== Domains and transitions =======================
% ----------- Mode 1 -----------
y = x{1};
hX{1} = [ 1 - y(1).^2;              % X_1 = [-1,1] x [0,1] x [-pi/2,pi/2]
          y(2) * (1-y(2));
          (pi/2)^2 - y(3).^2 ];
hU{1} = [ ua(1) * (1 - ua(1));      % U_1 = [0,1] x [-1,1]
          1 - ua(2)^2 ];
% transition 1->3
sX{1,3} = [ y(2) - 1;               % S_13 = [-1,1] x {1} x [-pi/2,pi/2]
            1 - y(2);
            hX{1} ];
R{1,3} = 1;                         % R_13 = 1
% transition 1->2
sX{1,2} = [ -y(2);                  % S_12 = [-1,1] x {0} x [-pi/2,pi/2]
            hX{1} ];
R{1,2} = y;                         % R_12 = Identity

% -------------- Mode 2 ----------------
y = x{2};
hX{2} = [ 1 - y(1).^2;              % X_2 = [-1,1] x [-1,0] x [-pi/2,pi/2]
          -y(2) * (y(2)+1);
          (pi/2)^2 - y(3).^2 ];
hU{2} = [ ua(1) * (1 - ua(1));      % U_2 = [0,1] x [-1,1]
          1 - ua(2)^2 ];

% -------------- Mode 3 ----------------
y = x{3};
hX{3} = 1 - y.^2;                   % X_3 = [-1,1]
hU{3} = ub(1) * (1 - ub(1));        % U_3 = [0,1]
% transition 3->2
sX{3,2} = - (y+1)^2;                % S_32 = {-1}
R{3,2} = [ 0.6; -0.8; 0 ];          % R_32 = (0.6,-0.8,0)

% ===================== Cost functions and target set ====================
h{1} = 1;
h{2} = 1;
h{3} = 1;
H{1} = 0;
H{2} = 0;
H{3} = 0;

y = x{2};
hXT{2} = [ - (y(1) - 0.8)^2;
           - (y(2) + 0.8)^2;
           (pi/2)^2 - y(3)^2 ];

% ============================== Options =================================
options.freeFinalTime = 1;
options.withTransition = 1;
options.svd_eps = 1e4;

% Solve
[out] = HybridOCPDualSolver_transition(t,x,u,f,g,hX,hU,sX,R,x0,hXT,h,H,d,dr,options);

pval = T * out.pval;
disp(['LMI ' int2str(d) ' lower bound = ' num2str(pval)]);

%% Plot
