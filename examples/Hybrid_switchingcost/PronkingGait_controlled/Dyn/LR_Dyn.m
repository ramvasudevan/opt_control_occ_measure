function out1 = LR_Dyn(in1,in2)
%LR_DYN
%    OUT1 = LR_DYN(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.0.
%    04-Sep-2018 13:56:09

U1 = in2(1,:);
U2 = in2(2,:);
Y2 = in1(1,:);
Y3 = in1(2,:);
Y4 = in1(3,:);
Y5 = in1(4,:);
Y6 = in1(5,:);
Y7 = in1(6,:);
Y9 = in1(8,:);
t2 = Y5+Y7;
t3 = Y5+Y9;
t4 = sin(Y5);
t5 = cos(t2);
t6 = U1.*2.0e1;
t7 = 1.0./t5;
t8 = t4.*(1.0./2.0);
t9 = cos(t3);
t10 = U2.*2.0e1;
t11 = 1.0./t9;
t12 = Y3+t8;
t20 = t11.*t12.*2.0e1;
t13 = t10-t20+2.0e1;
t14 = Y3-t8;
t15 = Y7.*2.0;
t16 = Y5.*2.0;
t17 = t15+t16;
t27 = t7.*t14.*2.0e1;
t18 = t6-t27+2.0e1;
t19 = t5.*t18;
t21 = t9.*t13;
t22 = t19+t21-1.0;
t23 = Y5.*3.0;
t24 = Y7.*3.0;
t25 = t23+t24;
t26 = sin(t2);
t28 = sin(t3);
t29 = Y7+t16;
t30 = sin(t29);
t31 = cos(Y7);
t32 = U1.*1.0e1;
t43 = t7.*t14.*1.0e1;
t33 = t32-t43+1.0e1;
t34 = cos(Y9);
t35 = U2.*1.0e1;
t45 = t11.*t12.*1.0e1;
t36 = t35-t45+1.0e1;
t37 = t34.*t36.*(8.0e2./8.37e2);
t44 = t31.*t33.*(8.0e2./8.37e2);
t38 = t37-t44;
t39 = t18.*t26;
t40 = t13.*t28;
t41 = t39+t40;
t42 = t16+t24;
t46 = Y6.^2;
t47 = cos(t29);
t48 = Y3.*2.0;
t49 = Y3.*Y6.*2.0;
t50 = Y5+t15;
t51 = sin(t50);
t52 = cos(t17);
t53 = Y2.*t52;
t54 = sin(t17);
t55 = Y4.*t54;
t58 = Y6.*t4.*(1.0./2.0);
t59 = Y6.*t51.*(1.0./2.0);
t56 = Y2+t49+t53+t55-t58-t59;
t61 = t4-t48;
t57 = 1.0./t61.^2;
t60 = t56.^2;
t62 = 1.0./t61;
t63 = Y9.*2.0;
t64 = t16+t63;
t65 = Y9.*3.0;
t66 = t23+t65;
t67 = Y9+t16;
t68 = sin(t67);
t69 = t16+t65;
t70 = t4+t48;
t71 = Y5+t63;
t72 = sin(t71);
t73 = Y6.*t72.*(1.0./2.0);
t74 = cos(t64);
t75 = Y2.*t74;
t76 = sin(t64);
t77 = Y4.*t76;
t78 = Y2+t49+t58+t73+t75+t77;
t79 = cos(t67);
t80 = 1.0./t70.^2;
t81 = t78.^2;
t82 = 1.0./t70;
out1 = [-t26.*(t6-t7.*(Y3-t4.*(1.0./2.0)).*2.0e1+2.0e1)-t13.*t28;Y4;t22;Y6;t38;t56./(t4-t48);-(t5.*t41.*-6.0+t22.*t26.*2.0-t30.*t38-t31.*t46.*6.0+t46.*t47-t41.*cos(t25).*2.0+t46.*cos(t42)+t22.*sin(t25).*2.0-t38.*sin(t42)+Y4.*Y6.*t5.*1.6e1+Y3.*t5.*t38.*8.0+Y3.*t26.*t46.*1.6e1-t31.*t57.*t60.*4.0+t47.*t57.*t60.*4.0+Y4.*t5.*t56.*t62.*1.6e1+Y3.*t26.*t57.*t60.*1.6e1-Y6.*t31.*t56.*t62.*1.2e1+Y6.*t47.*t56.*t62.*4.0+Y3.*Y6.*t26.*t56.*t62.*3.2e1)./(t30.*-2.0+sin(Y7).*2.0+Y3.*t5.*8.0);-t78.*t82;-(t9.*t41.*-6.0+t22.*t28.*2.0+t34.*t46.*6.0+t38.*t68-t46.*t79-t41.*cos(t66).*2.0-t46.*cos(t69)+t22.*sin(t66).*2.0+t38.*sin(t69)+Y4.*Y6.*t9.*1.6e1+Y3.*t9.*t38.*8.0+Y3.*t28.*t46.*1.6e1+t34.*t80.*t81.*4.0-t79.*t80.*t81.*4.0-Y4.*t9.*t78.*t82.*1.6e1+Y3.*t28.*t80.*t81.*1.6e1-Y6.*t34.*t78.*t82.*1.2e1+Y6.*t78.*t79.*t82.*4.0-Y3.*Y6.*t28.*t78.*t82.*3.2e1)./(t68.*2.0-sin(Y9).*2.0+Y3.*t9.*8.0)];
