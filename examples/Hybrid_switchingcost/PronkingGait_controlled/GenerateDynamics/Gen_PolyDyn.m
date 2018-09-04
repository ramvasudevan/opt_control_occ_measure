% Generate real dynamics and polynomial dynamics for 4 modes

d = 4;

% x0 = zeros( 10, 1 );
x0 = [ 1, 1, 0, 0, 0, 0, 0, 0, 0 ];      % Taylor expansion point for x
u0 = [ 0, 0 ];                           % Taylor expansion point for u

[dydt00, Y, U] = Gen_Dyn_helper( 0, 0 );       % Flight phase
dydt01 = Gen_Dyn_helper( 0, 1 );       % R-stance
dydt10 = Gen_Dyn_helper( 1, 0 );       % L-stance
dydt11 = Gen_Dyn_helper( 1, 1 );       % Double support

Y = Y(2:end);

f00 = taylor( subs(dydt00(2:end), U, u0), [Y], 'Order', d+1, 'ExpansionPoint', x0 );
f01 = taylor( subs(dydt01(2:end), U, u0), [Y], 'Order', d+1, 'ExpansionPoint', x0 );
f10 = taylor( subs(dydt10(2:end), U, u0), [Y], 'Order', d+1, 'ExpansionPoint', x0 );
f11 = taylor( subs(dydt11(2:end), U, u0), [Y], 'Order', d+1, 'ExpansionPoint', x0 );

g00 = taylor( subs(dydt00(2:end), Y, x0), [Y], 'Order', d+1, 'ExpansionPoint', [x0, u0] );
g01 = taylor( subs(dydt01(2:end), Y, x0), [Y], 'Order', d+1, 'ExpansionPoint', [x0, u0] );
g10 = taylor( subs(dydt10(2:end), Y, x0), [Y], 'Order', d+1, 'ExpansionPoint', [x0, u0] );
g11 = taylor( subs(dydt11(2:end), Y, x0), [Y], 'Order', d+1, 'ExpansionPoint', [x0, u0] );

matlabFunction( dydt00(2:end), 'Vars', {Y, U}, 'File', 'Dyn/FF_Dyn' );
matlabFunction( dydt01(2:end), 'Vars', {Y, U}, 'File', 'Dyn/FR_Dyn' );
matlabFunction( dydt10(2:end), 'Vars', {Y, U}, 'File', 'Dyn/LF_Dyn' );
matlabFunction( dydt11(2:end), 'Vars', {Y, U}, 'File', 'Dyn/LR_Dyn' );

hf00 = matlabFunction( f00, 'Vars', {Y, U}, 'File', 'PolyDyn/FF_Dyn_poly' );
hf01 = matlabFunction( f01, 'Vars', {Y, U}, 'File', 'PolyDyn/FR_Dyn_poly' );
hf10 = matlabFunction( f10, 'Vars', {Y, U}, 'File', 'PolyDyn/LF_Dyn_poly' );
% hf11 = matlabFunction( f11, 'Vars', {Y}, 'File', 'PolyDyn/LR_Dyn_poly' );
