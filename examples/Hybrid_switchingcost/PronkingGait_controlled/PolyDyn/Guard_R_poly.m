function out1 = Guard_R_poly(in1)
%GUARD_R_POLY
%    OUT1 = GUARD_R_POLY(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    04-Sep-2018 17:19:08

Y3 = in1(3,:);
Y4 = in1(4,:);
Y5 = in1(5,:);
Y6 = in1(6,:);
Y9 = in1(9,:);
Y10 = in1(10,:);
t2 = Y9.^2;
t3 = Y5.^2;
out1 = [Y3+Y5.*(1.0./2.0)+t2.*(1.0./2.0)+t3.*(1.0./2.0)+Y5.*Y9-Y5.*t3.*(1.0./1.2e1)-t2.*t3.*(1.0./4.0)-t2.^2.*(1.0./2.4e1)-t3.^2.*(1.0./2.4e1)-Y5.*Y9.*t2.*(1.0./6.0)-Y5.*Y9.*t3.*(1.0./6.0)-1.0;Y4+Y6.*(1.0./2.0)+Y5.*Y6+Y5.*Y10+Y6.*Y9+Y9.*Y10-Y6.*t3.*(1.0./4.0)-Y5.*Y6.*t2.*(1.0./2.0)-Y5.*Y6.*t3.*(1.0./6.0)-Y5.*Y10.*t2.*(1.0./2.0)-Y6.*Y9.*t2.*(1.0./6.0)-Y5.*Y10.*t3.*(1.0./6.0)-Y6.*Y9.*t3.*(1.0./2.0)-Y9.*Y10.*t2.*(1.0./6.0)-Y9.*Y10.*t3.*(1.0./2.0)];
