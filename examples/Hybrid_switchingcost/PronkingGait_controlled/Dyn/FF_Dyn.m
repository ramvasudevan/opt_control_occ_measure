function out1 = FF_Dyn(in1,in2)
%FF_DYN
%    OUT1 = FF_DYN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    04-Sep-2018 13:56:06

Y4 = in1(3,:);
Y6 = in1(5,:);
Y7 = in1(6,:);
Y8 = in1(7,:);
Y9 = in1(8,:);
Y10 = in1(9,:);
t2 = Y6.^2;
out1 = [0.0;Y4;-1.0;Y6;0.0;Y8;Y7.*-5.0-t2.*cos(Y7).*(1.0./2.0);Y10;Y9.*-5.0+t2.*cos(Y9).*(1.0./2.0)];
