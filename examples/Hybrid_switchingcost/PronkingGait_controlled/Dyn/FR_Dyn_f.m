function f01 = FR_Dyn_f(in1)
%FR_DYN_F
%    F01 = FR_DYN_F(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.1.
%    04-Sep-2018 22:38:24

Y2 = in1(2,:);
Y3 = in1(3,:);
Y4 = in1(4,:);
Y5 = in1(5,:);
Y6 = in1(6,:);
Y7 = in1(7,:);
Y8 = in1(8,:);
Y9 = in1(9,:);
t2 = Y5+Y9;
t3 = cos(t2);
t4 = 1.0./t3;
t5 = sin(Y5);
t6 = t5.*(1.0./2.0);
t7 = Y3+t6;
t8 = t4.*t7.*2.0e1;
t9 = t8-2.0e1;
t10 = cos(Y9);
t11 = t4.*t7.*1.0e1;
t12 = t11-1.0e1;
t13 = sin(t2);
t14 = Y5+Y7;
t15 = Y9.*2.0;
t16 = Y5.*2.0;
t17 = t15+t16;
t18 = Y6.^2;
t19 = Y9.*3.0;
t20 = t3.*t9;
t21 = t20+1.0;
t22 = Y9+t16;
t23 = sin(t22);
t24 = Y5.*3.0;
t25 = t19+t24;
t26 = Y3.*2.0;
t27 = t5+t26;
t28 = Y3.*Y6.*2.0;
t29 = Y6.*t5.*(1.0./2.0);
t30 = Y5+t15;
t31 = sin(t30);
t32 = Y6.*t31.*(1.0./2.0);
t33 = cos(t17);
t34 = Y2.*t33;
t35 = sin(t17);
t36 = Y4.*t35;
t37 = Y2+t28+t29+t32+t34+t36;
t38 = cos(t22);
t39 = 1.0./t27.^2;
t40 = t37.^2;
t41 = t16+t19;
t42 = 1.0./t27;
f01 = [Y2;t9.*t13;Y4;-t3.*t9-1.0;Y6;t10.*t12.*(-8.0e2./8.37e2);Y8;Y7.*-5.0+t10.*t12.*(8.0e2./8.37e2)-t18.*cos(Y7).*(1.0./2.0)-t10.*t12.*sin(Y7).*(4.0e2./8.37e2)-t9.*t13.*cos(t14)+t3.*t9.*sin(t14);-t37.*t42;(t10.*t18.*-6.0+t13.*t21.*2.0+t18.*t38+t18.*cos(t41)+t21.*sin(t25).*2.0-t9.*t13.*cos(t25).*2.0+t10.*t12.*sin(t41).*(8.0e2./8.37e2)-Y4.*Y6.*t3.*1.6e1-Y3.*t13.*t18.*1.6e1-t3.*t9.*t13.*6.0+t10.*t12.*t23.*(8.0e2./8.37e2)-t10.*t39.*t40.*4.0+t38.*t39.*t40.*4.0+Y3.*t3.*t10.*t12.*7.646356033452808+Y4.*t3.*t37.*t42.*1.6e1-Y3.*t13.*t39.*t40.*1.6e1+Y6.*t10.*t37.*t42.*1.2e1-Y6.*t37.*t38.*t42.*4.0+Y3.*Y6.*t13.*t37.*t42.*3.2e1)./(t23.*2.0-sin(Y9).*2.0+Y3.*t3.*8.0)];