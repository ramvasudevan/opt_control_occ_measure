function out1 = LF_Dyn(in1,in2)
%LF_DYN
%    OUT1 = LF_DYN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    04-Sep-2018 13:56:07

U1 = in2(1,:);
Y2 = in1(1,:);
Y3 = in1(2,:);
Y4 = in1(3,:);
Y5 = in1(4,:);
Y6 = in1(5,:);
Y7 = in1(6,:);
Y9 = in1(8,:);
Y10 = in1(9,:);
t2 = Y5+Y7;
t3 = cos(t2);
t4 = U1.*2.0e1;
t5 = 1.0./t3;
t6 = sin(Y5);
t9 = t6.*(1.0./2.0);
t7 = Y3-t9;
t13 = t5.*t7.*2.0e1;
t8 = t4-t13+2.0e1;
t10 = Y7.*2.0;
t11 = Y5.*2.0;
t12 = t10+t11;
t14 = t3.*t8;
t15 = t14-1.0;
t16 = sin(t2);
t17 = Y7.*3.0;
t18 = Y6.^2;
t19 = cos(Y7);
t20 = Y7+t11;
t21 = cos(t20);
t22 = Y3.*2.0;
t23 = Y3.*Y6.*2.0;
t24 = Y5+t10;
t25 = sin(t24);
t26 = cos(t12);
t27 = Y2.*t26;
t28 = sin(t12);
t29 = Y4.*t28;
t39 = Y6.*t6.*(1.0./2.0);
t40 = Y6.*t25.*(1.0./2.0);
t30 = Y2+t23+t27+t29-t39-t40;
t31 = sin(t20);
t32 = U1.*1.0e1;
t37 = t5.*t7.*1.0e1;
t33 = t32-t37+1.0e1;
t34 = Y5.*3.0;
t35 = t17+t34;
t36 = t11+t17;
t42 = t6-t22;
t38 = 1.0./t42.^2;
t41 = t30.^2;
t43 = 1.0./t42;
t44 = Y5+Y9;
out1 = [-t8.*t16;Y4;t15;Y6;t19.*t33.*(-8.0e2./8.37e2);t30./(t6-t22);-(t15.*t16.*2.0-t18.*t19.*6.0+t18.*t21+t18.*cos(t36)+t15.*sin(t35).*2.0-t8.*t16.*cos(t35).*2.0+t19.*t33.*sin(t36).*(8.0e2./8.37e2)+Y4.*Y6.*t3.*1.6e1+Y3.*t16.*t18.*1.6e1-t3.*t8.*t16.*6.0+t19.*t31.*t33.*(8.0e2./8.37e2)-t19.*t38.*t41.*4.0+t21.*t38.*t41.*4.0-Y3.*t3.*t19.*t33.*7.646356033452808+Y4.*t3.*t30.*t43.*1.6e1+Y3.*t16.*t38.*t41.*1.6e1-Y6.*t19.*t30.*t43.*1.2e1+Y6.*t21.*t30.*t43.*4.0+Y3.*Y6.*t16.*t30.*t43.*3.2e1)./(t31.*-2.0+sin(Y7).*2.0+Y3.*t3.*8.0);Y10;Y9.*-5.0+t19.*t33.*(8.0e2./8.37e2)+t18.*cos(Y9).*(1.0./2.0)+t19.*t33.*sin(Y9).*(4.0e2./8.37e2)+t8.*t16.*cos(t44)-t3.*t8.*sin(t44)];
