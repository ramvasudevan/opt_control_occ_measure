function f10 = LF_Dyn_poly(in1)
%LF_DYN_POLY
%    F10 = LF_DYN_POLY(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    27-Feb-2018 14:41:22

Y2 = in1(1,:);
Y3 = in1(2,:);
Y4 = in1(3,:);
Y5 = in1(4,:);
Y6 = in1(5,:);
Y7 = in1(6,:);
Y9 = in1(8,:);
Y10 = in1(9,:);
t2 = Y3-1.0;
t3 = Y7.^2;
t4 = Y5.^2;
t5 = t4.^2;
t6 = t3.^2;
t7 = t2.^2;
t8 = Y2-1.0;
t9 = Y3.*9.55794504181601;
t10 = Y6.^2;
t11 = Y4.^2;
t12 = t3.*4.778972520908005;
t13 = t8.^2;
t14 = Y7.*t2.*2.0e1;
t15 = Y9.^2;
t16 = t4.*4.778972520908005;
t17 = t5.*1.991238550378335;
t18 = Y7.*t3.*1.0e1;
t19 = t6.*3.982477100756671e-1;
t20 = t2.*t4.*4.778972520908005;
t21 = Y7.*t2.*t3.*(2.0e1./3.0);
t22 = Y5.*t2.*t3.*2.0e1;
t23 = Y5.*Y7.*t2.*9.55794504181601;
f10 = [t4.*-1.0e1-t5.*(5.0./3.0)+t14+t18+t21+t22-Y5.*Y7.*1.0e1+Y5.*t2.*2.0e1+Y5.*t3.*3.0e1+Y5.*t4.*1.0e1+Y7.*t4.*3.0e1-t3.*t4.*1.0e1-Y5.*Y7.*t3.*(1.0e1./3.0)-Y5.*Y7.*t4.*(2.5e1./3.0)+Y5.*t2.*t4.*(2.0e1./3.0)+Y7.*t2.*t4.*2.0e1;Y4;Y3.*-2.0e1+Y5.*1.0e1-t3.*1.0e1-t4.*1.0e1+t5.*(5.0./6.0)+t6.*(5.0./6.0)-Y5.*Y7.*2.0e1-Y5.*t4.*(5.0./3.0)+t3.*t4.*5.0+Y5.*Y7.*t3.*(1.0e1./3.0)+Y5.*Y7.*t4.*(1.0e1./3.0)+1.9e1;Y6;Y5.*(-4.778972520908005)-t6.*3.982477100756671e-1+t9+t12+t16+t17+t20+t23+Y5.*Y7.*9.55794504181601-Y5.*t4.*1.592990840302668-Y7.*t4.*4.778972520908005+t3.*t4.*9.55794504181601+Y5.*Y7.*t3.*3.185981680605337+Y5.*Y7.*t4.*7.964954201513341-9.55794504181601;-Y2+Y3-Y5.*(1.0./2.0)-Y6+t3+t4.*(3.0./4.0)-t5.*(1.0./1.6e1)-t6.*(1.0./3.0)-t7-Y4.*Y5-Y4.*Y7+Y5.*Y7.*2.0+Y6.*Y7.*(1.0./2.0)+Y5.*t2-Y4.*t4.*(1.0./2.0)+Y5.*t3.*(1.0./2.0)+Y5.*t4.*(1.1e1./2.4e1)+Y7.*t4-Y5.*t7.*(3.0./2.0)-Y5.*t8.*(1.0./2.0)-t2.*t3-t2.*t4.*(1.0./4.0)-t3.*t4.*(7.0./4.0)+t2.*t7+t2.*t8+t3.*t7+t3.*t8-t4.*t7.*(1.0./2.0)+t4.*t8.*(3.0./4.0)-t7.*t8-t7.^2-Y4.*Y5.*Y7.*(1.0./2.0)+Y5.*Y6.*Y7.*(1.0./4.0)+Y4.*Y5.*t2+Y4.*Y5.*t3.*2.0+Y4.*Y5.*t4.*(5.0./1.2e1)+Y4.*Y7.*t2+Y4.*Y7.*t3.*(2.0./3.0)-Y5.*Y6.*t3.*(1.0./2.0)-Y5.*Y7.*t2.*2.0+Y4.*Y7.*t4.*(7.0./4.0)-Y5.*Y7.*t3.*(4.0./3.0)-Y6.*Y7.*t2.*(1.0./2.0)-Y4.*Y5.*t7-Y5.*Y7.*t4.*(5.0./6.0)-Y6.*Y7.*t3.*(1.0./3.0)-Y6.*Y7.*t4.*(1.0./8.0)-Y4.*Y7.*t7+Y5.*Y7.*t7.*2.0+Y5.*Y7.*t8.*2.0+Y6.*Y7.*t7.*(1.0./2.0)+Y4.*t2.*t4-Y5.*t2.*t3-Y5.*t2.*t4.*(2.0./3.0)-Y7.*t2.*t4.*2.0+Y5.*t2.*t7.*2.0+Y5.*t2.*t8+Y5.*t3.*t8.*(1.0./2.0)+Y5.*t4.*t8.*(1.1e1./2.4e1)+Y7.*t4.*t8-Y5.*t7.*t8.*(3.0./2.0)-t2.*t3.*t8-t2.*t4.*t8.*(1.0./4.0)+t2.*t7.*t8+Y4.*Y5.*Y7.*t2-Y5.*Y6.*Y7.*t2.*(1.0./2.0)-Y5.*Y7.*t2.*t8.*2.0-1.0;Y4.*2.0+Y5.*3.778972520908005-Y6-Y7-t4.*6.278972520908005+t5.*3.837614496216647e-1-t9-t10.*(1.0./2.0)-t12+t19+Y4.*Y5.*2.0-Y5.*Y6-Y5.*Y7.*1.344743130227001e1-Y4.*t2.*4.0-Y4.*t3.*6.0+Y5.*t2.*3.0-Y4.*t4.*(9.0./2.0)+Y5.*t3.*1.2778972520908e1+Y6.*t2.*2.0+Y5.*t4.*3.009657506969335+Y6.*t3.*3.0+Y7.*t2.*7.778972520908005+Y6.*t4.*(3.0./4.0)+Y7.*t3.*5.056152927120669+Y4.*t7.*6.0+Y7.*t4.*1.272371565113501e1+Y4.*t8.*2.0-Y5.*t7.*5.0-Y5.*t8.*4.0-Y6.*t7.*3.0-Y6.*t8-Y7.*t7.*9.778972520908005-Y5.*t10.*(1.0./4.0)-Y7.*t8.*4.0+Y5.*t11.*2.0+Y7.*t10.*(1.0./2.0)-Y5.*t13.*2.0+Y7.*t11.*2.0-Y7.*t13.*2.0+t2.*t4.*(1.85e2./8.37e2)+t3.*t4.*(5.11e2./2.79e2)-t4.*t7.*(2.1e1./2.0)+t2.*t10.*(1.0./2.0)-t4.*t8.*4.0+t3.*t10.*(1.0./2.0)+t4.*t10.*(1.0./8.0)+t4.*t11.*2.0-t4.*t13.*2.0-t7.*t10.*(1.0./2.0)-Y4.*Y5.*Y6-Y4.*Y5.*Y7.*1.2e1-Y4.*Y6.*Y7.*2.0+Y5.*Y6.*Y7.*4.0-Y4.*Y5.*t2.*6.0-Y4.*Y5.*t3.*6.0-Y4.*Y5.*t4.*(1.6e1./3.0)+Y5.*Y6.*t2.*3.0-Y4.*Y6.*t4+Y5.*Y6.*t3.*3.0+Y5.*Y7.*t2.*(1.85e2./8.37e2)-Y4.*Y7.*t4.*1.2e1+Y5.*Y6.*t4.*(7.0./6.0)+Y5.*Y7.*t3.*2.601752289924333+Y4.*Y5.*t7.*1.2e1+Y5.*Y7.*t4.*1.405665073675826+Y4.*Y5.*t8.*2.0+Y6.*Y7.*t4.*4.0-Y5.*Y6.*t7.*6.0-Y5.*Y6.*t8-Y5.*Y7.*t7.*1.766845878136201e1-Y5.*Y7.*t8.*4.0+Y5.*Y7.*t10+Y5.*Y7.*t11.*2.0-Y5.*Y7.*t13.*2.0+Y4.*t2.*t3.*1.2e1+Y4.*t2.*t4.*6.0-Y5.*t2.*t3.*2.2778972520908e1-Y5.*t2.*t4.*(3.0./4.0)-Y6.*t2.*t3.*6.0-Y7.*t2.*t3.*1.157546794105934e1-Y4.*t2.*t7.*8.0-Y7.*t2.*t4.*1.1555256869773e1-Y4.*t2.*t8.*4.0+Y5.*t2.*t7.*7.0-Y4.*t3.*t8.*6.0+Y5.*t2.*t8.*8.0+Y6.*t2.*t7.*4.0-Y4.*t4.*t8.*(9.0./2.0)+Y5.*t3.*t8.*2.0e1+Y6.*t2.*t8.*2.0+Y7.*t2.*t7.*1.1778972520908e1+Y5.*t2.*t10.*(1.0./2.0)+Y5.*t4.*t8.*(1.1e1./3.0)+Y6.*t3.*t8.*3.0+Y7.*t2.*t8.*8.0-Y5.*t2.*t11.*4.0+Y6.*t4.*t8.*(3.0./4.0)+Y7.*t3.*t8.*(2.0e1./3.0)+Y4.*t7.*t8.*6.0-Y7.*t2.*t10+Y7.*t4.*t8.*1.7e1+Y5.*t2.*t13.*4.0-Y5.*t7.*t8.*1.2e1-Y7.*t2.*t11.*4.0-Y6.*t7.*t8.*3.0+Y7.*t2.*t13.*4.0-Y7.*t7.*t8.*1.2e1+t2.*t4.*t8.*1.2e1-Y4.*Y5.*Y6.*Y7.*2.0+Y4.*Y5.*Y6.*t2.*2.0+Y4.*Y5.*Y7.*t2.*2.4e1+Y4.*Y6.*Y7.*t2.*4.0-Y5.*Y6.*Y7.*t2.*8.0-Y4.*Y5.*Y7.*t8.*1.2e1+Y5.*Y6.*Y7.*t8.*4.0-Y4.*Y5.*t2.*t8.*6.0+Y5.*Y6.*t2.*t8.*3.0+Y5.*Y7.*t2.*t8.*1.2e1+9.55794504181601;Y10;Y5.*4.778972520908005-Y9.*5.0-t9+t10.*(1.0./2.0)-t12-t14-t16-t17-t18+t19-t20-t21-t22-t23+Y5.*Y7.*(3.7e2./8.37e2)-Y5.*Y9.*7.610513739545998-Y5.*t3.*2.0e1+Y5.*t4.*1.592990840302668-Y7.*t4.*5.221027479091995+Y9.*t2.*1.5221027479092e1+Y9.*t3.*7.610513739545998+Y9.*t4.*7.610513739545998+t3.*t4.*(3.7e2./8.37e2)-t10.*t15.*(1.0./4.0)+Y5.*Y7.*Y9.*1.5221027479092e1+Y5.*Y7.*t3.*1.473516527279968e-1-Y5.*Y7.*t4.*4.631620868180008-Y5.*Y9.*t4.*2.536837913181999-Y7.*Y9.*t4.*7.610513739545998-Y5.*Y7.*t15.*5.0+Y5.*Y9.*t15.*1.268418956591-Y7.*t2.*t4.*1.0e1+Y9.*t2.*t4.*7.610513739545998+Y7.*t2.*t15.*1.0e1-Y9.*t2.*t15.*2.536837913181999+Y5.*Y7.*Y9.*t2.*1.5221027479092e1+9.55794504181601];
