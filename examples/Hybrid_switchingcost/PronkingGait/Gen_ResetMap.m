% Generate real and polynomial guards/reset maps

% Define model parameters:
    k  = 20;
    la = 0.5;
    l = 1;
    ks = 5; % omega = sqrt(5);
    M = 1;
    J = 1.04625;%1.5;%1.046875;% 0.3; % 0.25;

    Y = sym('Y',[10,1]);
    
    x           = Y(1);
    dx          = Y(2);
    y           = Y(3);
    dy          = Y(4);
    phi         = Y(5);
    dphi        = Y(6);
    alpha_L     = Y(7);
    dalpha_L    = Y(8);
    alpha_R     = Y(9);
    dalpha_R    = Y(10);
    
    d = 4;
    x0 = [ 1, 1, 0, 0, 0, 0, 0, 0, 0 ];        % Taylor expansion point
%% Guards
% Left (rear) leg - TD:
% y - la * sin( phi ) - l * cos( phi + alpha_L ) -> 0+
Guard_L_TD = y - la * sin( phi ) - l * cos( phi + alpha_L );

% Right (Front) leg - TD:
% y + la * sin( phi ) - l * cos( phi + alpha_R ) -> 0+
Guard_R_TD = y + la * sin( phi ) - l * cos( phi + alpha_R );

% Left (rear) leg - TD - derivative:
% dy - la*cos(phi)*dphi + l*sin(phi+alpha_L)*(dphi+alpha_L) < 0
dGuard_L_TD = dy - la * cos( phi ) * dphi + l * sin( phi + alpha_L ) * ( dphi + dalpha_L );

% Right (front) leg - TD - derivative:
% dy + la*cos(phi)*dphi + l*sin(phi+alpha_R)*(dphi+alpha_R) < 0
dGuard_R_TD = dy + la * cos( phi ) * dphi + l * sin( phi + alpha_R ) * ( dphi + dalpha_R );

%% Reset maps
[dydt, Y] = Gen_Dyn_helper( 1, 1 );
Reset_L_TD = Y;
Reset_L_TD( 8 ) = dydt( 7 );

Reset_R_TD = Y;
Reset_R_TD( 10 ) = dydt( 9 );

%% Export guards (event functions) and reset maps
Y = Y(2:end);
Reset_L_TD = Reset_L_TD( 2:end );
Reset_R_TD = Reset_R_TD( 2:end );

matlabFunction( Guard_L_TD, 'Vars', {Y}, 'File', 'Dyn/Guard_L' );
matlabFunction( Guard_R_TD, 'Vars', {Y}, 'File', 'Dyn/Guard_R' );

matlabFunction( Reset_L_TD, 'Vars', {Y}, 'File', 'Dyn/Reset_L' );
matlabFunction( Reset_R_TD, 'Vars', {Y}, 'File', 'Dyn/Reset_R' );

% ====================== Note ===========================
% 1. Guards for lift-offs are the same as the ones for touch-downs, but
% they have to be approached from the opposite direction.
% 2. Reset maps for lift-offs are identity maps.

%% Polynomial guards/reset maps
Guard_L_TD_poly = taylor( Guard_L_TD, Y, 'Order', d+1, 'ExpansionPoint', x0 );
dGuard_L_TD_poly = taylor( dGuard_L_TD, Y, 'Order', d+1, 'ExpansionPoint', x0 );
Guard_R_TD_poly = taylor( Guard_R_TD, Y, 'Order', d+1, 'ExpansionPoint', x0 );
dGuard_R_TD_poly = taylor( dGuard_R_TD, Y, 'Order', d+1, 'ExpansionPoint', x0 );

Reset_L_TD_poly = taylor( Reset_L_TD, Y, 'Order', d+1, 'ExpansionPoint', x0 );
Reset_R_TD_poly = taylor( Reset_R_TD, Y, 'Order', d+1, 'ExpansionPoint', x0 );

matlabFunction( [ Guard_L_TD_poly; dGuard_L_TD_poly ], 'Vars', {Y}, 'File', 'PolyDyn/Guard_L_poly' );
matlabFunction( [ Guard_R_TD_poly; dGuard_R_TD_poly ], 'Vars', {Y}, 'File', 'PolyDyn/Guard_R_poly' );

matlabFunction( Reset_L_TD_poly, 'Vars', {Y}, 'File', 'PolyDyn/Reset_L_poly' );
matlabFunction( Reset_R_TD_poly, 'Vars', {Y}, 'File', 'PolyDyn/Reset_R_poly' );



