function g01 = FR_Dyn_g(in1)
%FR_DYN_G
%    G01 = FR_DYN_G(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    04-Sep-2018 22:38:26

Y3 = in1(3,:);
Y5 = in1(5,:);
Y7 = in1(7,:);
Y9 = in1(9,:);
t2 = Y5+Y9;
t3 = cos(Y9);
t4 = t3.*9.55794504181601;
t5 = sin(t2);
t6 = cos(t2);
t7 = Y5+Y7;
t8 = Y5.*2.0;
t9 = Y9+t8;
t10 = sin(t9);
t11 = Y5.*3.0;
t12 = Y9.*3.0;
t13 = t11+t12;
g01 = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t5.*-2.0e1,0.0,t6.*2.0e1,0.0,t4,0.0,-t4+t3.*sin(Y7).*4.778972520908005+t5.*cos(t7).*2.0e1-t6.*sin(t7).*2.0e1,0.0,-(t5.*t6.*-8.0e1+t3.*t10.*9.55794504181601+t3.*sin(t8+t12).*9.55794504181601-t5.*cos(t13).*4.0e1+t6.*sin(t13).*4.0e1+Y3.*t3.*t6.*7.646356033452808e1)./(t10.*2.0-sin(Y9).*2.0+Y3.*t6.*8.0)],[10,2]);
