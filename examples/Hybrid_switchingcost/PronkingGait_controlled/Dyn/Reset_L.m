function Reset_L_TD = Reset_L(in1)
%RESET_L
%    RESET_L_TD = RESET_L(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    04-Sep-2018 17:19:07

Y1 = in1(1,:);
Y2 = in1(2,:);
Y3 = in1(3,:);
Y4 = in1(4,:);
Y5 = in1(5,:);
Y6 = in1(6,:);
Y7 = in1(7,:);
Y9 = in1(9,:);
Y10 = in1(10,:);
t2 = Y3-1.0;
t3 = t2.^2;
t4 = Y2-1.0;
t5 = Y5.^2;
t6 = Y7.^2;
Reset_L_TD = [Y1;Y2;Y3;Y4;Y5;Y6;Y7;-Y2+Y3-Y5.*(1.0./2.0)-Y6-t3+t5.*(3.0./4.0)+t6-Y4.*Y5-Y4.*Y7+Y5.*Y7.*2.0+Y6.*Y7.*(1.0./2.0)+Y5.*t2-Y5.*t3.*(3.0./2.0)-Y4.*t5.*(1.0./2.0)-Y5.*t4.*(1.0./2.0)+Y5.*t5.*(1.1e1./2.4e1)+Y5.*t6.*(1.0./2.0)+Y7.*t5+t2.*t3+t2.*t4-t2.*t5.*(1.0./4.0)-t3.*t4-t2.*t6-t3.*t5.*(1.0./2.0)+t3.*t6+t4.*t5.*(3.0./4.0)+t4.*t6-t5.*t6.*(7.0./4.0)-t3.^2-t5.^2.*(1.0./1.6e1)-t6.^2.*(1.0./3.0)-Y4.*Y5.*Y7.*(1.0./2.0)+Y5.*Y6.*Y7.*(1.0./4.0)+Y4.*Y5.*t2-Y4.*Y5.*t3+Y4.*Y7.*t2+Y4.*Y5.*t5.*(5.0./1.2e1)-Y4.*Y7.*t3-Y5.*Y7.*t2.*2.0+Y4.*Y5.*t6.*2.0+Y5.*Y7.*t3.*2.0-Y6.*Y7.*t2.*(1.0./2.0)+Y4.*Y7.*t5.*(7.0./4.0)+Y5.*Y7.*t4.*2.0+Y6.*Y7.*t3.*(1.0./2.0)+Y4.*Y7.*t6.*(2.0./3.0)-Y5.*Y6.*t6.*(1.0./2.0)-Y5.*Y7.*t5.*(5.0./6.0)-Y5.*Y7.*t6.*(4.0./3.0)-Y6.*Y7.*t5.*(1.0./8.0)-Y6.*Y7.*t6.*(1.0./3.0)+Y5.*t2.*t3.*2.0+Y4.*t2.*t5+Y5.*t2.*t4-Y5.*t2.*t5.*(2.0./3.0)-Y5.*t3.*t4.*(3.0./2.0)-Y5.*t2.*t6+Y5.*t4.*t5.*(1.1e1./2.4e1)-Y7.*t2.*t5.*2.0+Y5.*t4.*t6.*(1.0./2.0)+Y7.*t4.*t5+t2.*t3.*t4-t2.*t4.*t5.*(1.0./4.0)-t2.*t4.*t6+Y4.*Y5.*Y7.*t2-Y5.*Y6.*Y7.*t2.*(1.0./2.0)-Y5.*Y7.*t2.*t4.*2.0-1.0;Y9;Y10];
