function f10 = LF_Dyn_f(in1)
%LF_DYN_F
%    F10 = LF_DYN_F(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    04-Sep-2018 22:38:24

Y2 = in1(2,:);
Y3 = in1(3,:);
Y4 = in1(4,:);
Y5 = in1(5,:);
Y6 = in1(6,:);
Y7 = in1(7,:);
Y9 = in1(9,:);
Y10 = in1(10,:);
t2 = Y5+Y7;
t3 = cos(t2);
t4 = 1.0./t3;
t5 = sin(Y5);
t9 = t5.*(1.0./2.0);
t6 = Y3-t9;
t7 = t4.*t6.*2.0e1;
t8 = t7-2.0e1;
t10 = Y7.*2.0;
t11 = Y5.*2.0;
t12 = t10+t11;
t13 = Y7.*3.0;
t14 = sin(t2);
t15 = t3.*t8;
t16 = t15+1.0;
t17 = Y6.^2;
t18 = cos(Y7);
t19 = Y7+t11;
t20 = cos(t19);
t21 = Y3.*2.0;
t22 = Y3.*Y6.*2.0;
t23 = Y5+t10;
t24 = sin(t23);
t25 = cos(t12);
t26 = Y2.*t25;
t27 = sin(t12);
t28 = Y4.*t27;
t37 = Y6.*t5.*(1.0./2.0);
t38 = Y6.*t24.*(1.0./2.0);
t29 = Y2+t22+t26+t28-t37-t38;
t30 = sin(t19);
t31 = t4.*t6.*1.0e1;
t32 = t31-1.0e1;
t33 = Y5.*3.0;
t34 = t13+t33;
t35 = t11+t13;
t40 = t5-t21;
t36 = 1.0./t40.^2;
t39 = t29.^2;
t41 = 1.0./t40;
t42 = t18.*t32.*(8.0e2./8.37e2);
t43 = Y5+Y9;
f10 = [Y2;t8.*t14;Y4;-t3.*t8-1.0;Y6;t42;t29./(t5-t21);-(t14.*t16.*-2.0-t17.*t18.*6.0+t17.*t20+t17.*cos(t35)-t16.*sin(t34).*2.0+t8.*t14.*cos(t34).*2.0-t18.*t32.*sin(t35).*(8.0e2./8.37e2)+Y4.*Y6.*t3.*1.6e1+Y3.*t14.*t17.*1.6e1+t3.*t8.*t14.*6.0-t18.*t30.*t32.*(8.0e2./8.37e2)-t18.*t36.*t39.*4.0+t20.*t36.*t39.*4.0+Y3.*t3.*t18.*t32.*7.646356033452808+Y4.*t3.*t29.*t41.*1.6e1+Y3.*t14.*t36.*t39.*1.6e1-Y6.*t18.*t29.*t41.*1.2e1+Y6.*t20.*t29.*t41.*4.0+Y3.*Y6.*t14.*t29.*t41.*3.2e1)./(t30.*-2.0+sin(Y7).*2.0+Y3.*t3.*8.0);Y10;Y9.*-5.0-t42+t17.*cos(Y9).*(1.0./2.0)-t18.*t32.*sin(Y9).*(4.0e2./8.37e2)-t8.*t14.*cos(t43)+t3.*t8.*sin(t43)];
