function g11 = LR_Dyn_g(in1)
%LR_DYN_G
%    G11 = LR_DYN_G(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    04-Sep-2018 22:38:26

Y3 = in1(3,:);
Y5 = in1(5,:);
Y7 = in1(7,:);
Y9 = in1(9,:);
t2 = Y5+Y7;
t3 = Y5+Y9;
t4 = cos(t2);
t5 = cos(Y7);
t6 = Y5.*2.0;
t7 = Y7+t6;
t8 = sin(t7);
t9 = sin(t2);
t10 = Y5.*3.0;
t11 = Y7.*3.0;
t12 = t10+t11;
t13 = sin(Y7);
t14 = t13.*2.0;
t15 = Y3.*t4.*8.0;
t16 = t8.*-2.0+t14+t15;
t17 = 1.0./t16;
t18 = cos(Y9);
t19 = cos(t12);
t20 = sin(t3);
t21 = sin(t12);
t22 = cos(t3);
t23 = t6+t11;
t24 = sin(t23);
t25 = Y9+t6;
t26 = sin(t25);
t27 = Y9.*3.0;
t28 = t10+t27;
t29 = sin(Y9);
t30 = t26.*2.0;
t31 = Y3.*t22.*8.0;
t32 = t29.*-2.0+t30+t31;
t33 = 1.0./t32;
t34 = cos(t28);
t35 = sin(t28);
t36 = t6+t27;
t37 = sin(t36);
g11 = reshape([0.0,t9.*-2.0e1,0.0,t4.*2.0e1,0.0,t5.*(-9.55794504181601),0.0,t17.*(t4.*t9.*8.0e1-t5.*t8.*9.55794504181601-t4.*t21.*4.0e1+t9.*t19.*4.0e1-t5.*t24.*9.55794504181601+Y3.*t4.*t5.*7.646356033452808e1),0.0,t33.*(t4.*t20.*-4.0e1+t5.*t26.*9.55794504181601+t9.*t22.*1.2e2-t4.*t35.*4.0e1+t5.*t37.*9.55794504181601+t9.*t34.*4.0e1+Y3.*t5.*t22.*7.646356033452808e1),0.0,t20.*-2.0e1,0.0,t22.*2.0e1,0.0,t18.*9.55794504181601,0.0,t17.*(t4.*t20.*1.2e2+t8.*t18.*9.55794504181601-t9.*t22.*4.0e1+t19.*t20.*4.0e1+t18.*t24.*9.55794504181601-t21.*t22.*4.0e1-Y3.*t4.*t18.*7.646356033452808e1),0.0,-t33.*(t20.*t22.*-8.0e1+t18.*t26.*9.55794504181601-t20.*t34.*4.0e1+t18.*t37.*9.55794504181601+t22.*t35.*4.0e1+Y3.*t18.*t22.*7.646356033452808e1)],[10,2]);