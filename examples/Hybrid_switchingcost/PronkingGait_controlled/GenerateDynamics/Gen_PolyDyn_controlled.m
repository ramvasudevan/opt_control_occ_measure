% Generate real dynamics and polynomial dynamics for 4 modes
clear;
clc;

d = 4;

% x0 = zeros( 10, 1 );
x0 = [ 0, 1, 1, 0, 0, 0, 0, 0, 0, 0 ];      % Taylor expansion point for x
u0 = [ 0, 0 ];                              % Taylor expansion point for u

[dydt00, Y, U] = Gen_Dyn_Ctrl_helper( 0, 0 );       % Flight phase
dydt01 = Gen_Dyn_Ctrl_helper( 0, 1 );       % R-stance
dydt10 = Gen_Dyn_Ctrl_helper( 1, 0 );       % L-stance
dydt11 = Gen_Dyn_Ctrl_helper( 1, 1 );       % Double support

f00 = subs( dydt00, U, u0' );
f01 = subs( dydt01, U, u0' );
f10 = subs( dydt10, U, u0' );
f11 = subs( dydt11, U, u0' );
g00 = [ diff( dydt00, U(1) ), diff( dydt00, U(2) ) ];
g01 = [ diff( dydt01, U(1) ), diff( dydt01, U(2) ) ];
g10 = [ diff( dydt10, U(1) ), diff( dydt10, U(2) ) ];
g11 = [ diff( dydt11, U(1) ), diff( dydt11, U(2) ) ];

%% True dynamics
matlabFunction( f00, 'Vars', {Y}, 'File', 'Dyn/FF_Dyn_f' );
matlabFunction( f01, 'Vars', {Y}, 'File', 'Dyn/FR_Dyn_f' );
matlabFunction( f10, 'Vars', {Y}, 'File', 'Dyn/LF_Dyn_f' );
matlabFunction( f11, 'Vars', {Y}, 'File', 'Dyn/LR_Dyn_f' );

matlabFunction( g00, 'Vars', {Y}, 'File', 'Dyn/FF_Dyn_g' );
matlabFunction( g01, 'Vars', {Y}, 'File', 'Dyn/FR_Dyn_g' );
matlabFunction( g10, 'Vars', {Y}, 'File', 'Dyn/LF_Dyn_g' );
matlabFunction( g11, 'Vars', {Y}, 'File', 'Dyn/LR_Dyn_g' );

%% Polynomial dynamics
f00_poly = taylor( f00, Y, 'Order', d+1, 'ExpansionPoint', x0 );
f01_poly = taylor( f01, Y, 'Order', d+1, 'ExpansionPoint', x0 );
f10_poly = taylor( f10, Y, 'Order', d+1, 'ExpansionPoint', x0 );
f11_poly = taylor( f11, Y, 'Order', d+1, 'ExpansionPoint', x0 );

g00_poly = taylor( g00, Y, 'Order', d+1, 'ExpansionPoint', x0 );
g01_poly = taylor( g01, Y, 'Order', d+1, 'ExpansionPoint', x0 );
g10_poly = taylor( g10, Y, 'Order', d+1, 'ExpansionPoint', x0 );
g11_poly = taylor( g11, Y, 'Order', d+1, 'ExpansionPoint', x0 );

matlabFunction( f00_poly, 'Vars', {Y}, 'File', 'PolyDyn/FF_Dyn_poly_f' );
matlabFunction( f01_poly, 'Vars', {Y}, 'File', 'PolyDyn/FR_Dyn_poly_f' );
matlabFunction( f10_poly, 'Vars', {Y}, 'File', 'PolyDyn/LF_Dyn_poly_f' );
matlabFunction( f11_poly, 'Vars', {Y}, 'File', 'PolyDyn/LR_Dyn_poly_f' );

matlabFunction( g00_poly, 'Vars', {Y}, 'File', 'PolyDyn/FF_Dyn_poly_g' );
matlabFunction( g01_poly, 'Vars', {Y}, 'File', 'PolyDyn/FR_Dyn_poly_g' );
matlabFunction( g10_poly, 'Vars', {Y}, 'File', 'PolyDyn/LF_Dyn_poly_g' );
matlabFunction( g11_poly, 'Vars', {Y}, 'File', 'PolyDyn/LR_Dyn_poly_g' );
