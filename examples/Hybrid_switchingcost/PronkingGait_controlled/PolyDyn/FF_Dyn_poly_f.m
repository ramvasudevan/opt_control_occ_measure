function f00_poly = FF_Dyn_poly_f(in1)
%FF_DYN_POLY_F
%    F00_POLY = FF_DYN_POLY_F(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    04-Sep-2018 22:38:29

Y2 = in1(2,:);
Y4 = in1(4,:);
Y6 = in1(6,:);
Y7 = in1(7,:);
Y8 = in1(8,:);
Y9 = in1(9,:);
Y10 = in1(10,:);
t2 = Y6.^2;
f00_poly = [Y2;0.0;Y4;-1.0;Y6;0.0;Y8;Y7.*-5.0-t2.*(1.0./2.0)+Y7.^2.*t2.*(1.0./4.0);Y10;Y9.*-5.0+t2.*(1.0./2.0)-Y9.^2.*t2.*(1.0./4.0)];