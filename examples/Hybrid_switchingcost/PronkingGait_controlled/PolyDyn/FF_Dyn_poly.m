function f00 = FF_Dyn_poly(in1,in2)
%FF_DYN_POLY
%    F00 = FF_DYN_POLY(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    04-Sep-2018 13:56:09

Y4 = in1(3,:);
Y6 = in1(5,:);
Y7 = in1(6,:);
Y8 = in1(7,:);
Y9 = in1(8,:);
Y10 = in1(9,:);
t2 = Y6.^2;
f00 = [0.0;Y4;-1.0;Y6;0.0;Y8;Y7.*-5.0-t2.*(1.0./2.0)+Y7.^2.*t2.*(1.0./4.0);Y10;Y9.*-5.0+t2.*(1.0./2.0)-Y9.^2.*t2.*(1.0./4.0)];
