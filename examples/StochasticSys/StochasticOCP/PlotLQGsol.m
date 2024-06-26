function [out] = PlotLQGsol( N, tvar, xvar, ctrl_law, v )
% inputs are (#trajs, mssvar t, mssvar x, msspoly u)
lw = 2;
ftsz = 18;

figure(1);

subplot(2,1,1);
ylabel('x','FontSize',ftsz);
xlabel('t','FontSize',ftsz);
hold on;
grid on;
box on;

subplot(2,1,2);
ylabel('u','FontSize',ftsz);
xlabel('t','FontSize',ftsz);
hold on;
grid on;
box on;

Jtotal1 = zeros(1,N);
Jtotal2 = zeros(1,N);
Jtotal3 = zeros(1,N);

for cnt = 1 : N
    %%%%%%% dx = ( a * x + b * u ) dt + sig * dw

    a = -1;
%     a = 0.05;
    b = 1;
    sig = 0.1;

    %%%%%%% J = E[ \int_t^T (0.5 * hx * x^2 + 0.5 * r * u^2 ) ds + 0.5 * HT * x(T)^2 ] 

    hx = 0;  
    r = 1;   
    HT = 1;

    %%%%%%% Initial Conditions

    t0 = 0;
    T  = 1;
    x0 = 0.3;


    %%%%%%% Wiener Process and Time

    NW = 1000;
    dt = T/NW;
    t = [t0 :dt:T]';
    dW = randn(1,NW)*sqrt(dt);

    %%%%%%% Analytically Optimal Solution: u^o = 2*x / ( 3 * exp(2*(1-t)) - 1 )
    x1 = zeros( 1, NW+1 );      u1 = zeros( 1, NW+1 );      % analytical
    x2 = zeros( 1, NW+1 );      u2 = zeros( 1, NW+1 );      % Pengcheng's u
    x3 = zeros( 1, NW+1 );      u2 = zeros( 1, NW+1 );      % u from Pengcheng's value function

    i = 1;
    x1(i) = x0;
    x2(i) = x0;
    x3(i) = x0;
    u1(i)= - 2*x1(i) / ( 3 * exp(2*(1-t(i))) - 1 );
    u2(i) = double( subs( ctrl_law, [tvar;xvar], [ t(i); x2(i) ] ) );
    u3(i) = double( subs(-diff(v,xvar), [tvar;xvar], [ t(i); x3(i)] ) );
    
    J1(i) = 0 + dt * (0.5 * hx * x1(i)^2 + 0.5 * r * u1(i)^2);
    J2(i) = 0 + dt * (0.5 * hx * x2(i)^2 + 0.5 * r * u2(i)^2);
    J3(i) = 0 + dt * (0.5 * hx * x3(i)^2 + 0.5 * r * u3(i)^2);

    for i = 2:NW+1
        x1(i) = x1(i-1) + (a * x1(i-1) + b * u1(i-1)) * dt + sig * dW(i-1);
        u1(i)= - 2*x1(i) / ( 3 * exp(2*(1-t(i))) - 1 );
        J1(i) = J1(i-1) + dt * (0.5 * hx * x1(i)^2 + 0.5 * r * u1(i)^2);
        
        x2(i) = x2(i-1) + (a * x2(i-1) + b * u2(i-1)) * dt + sig * dW(i-1);
        u2(i) = double( subs( ctrl_law, [tvar;xvar], [ t(i); x2(i) ] ) );
        J2(i) = J2(i-1) + dt * (0.5 * hx * x2(i)^2 + 0.5 * r * u2(i)^2);
        
        x3(i) = x3(i-1) + (a * x3(i-1) + b * u3(i-1)) * dt + sig * dW(i-1);
        u3(i) = double( subs( -diff(v,xvar), [tvar;xvar], [ t(i); x3(i) ] ) );
        J3(i) = J3(i-1) + dt * (0.5 * hx * x3(i)^2 + 0.5 * r * u3(i)^2);
    end

    Jtotal1(cnt) = J1(end) + 0.5 * HT * x1(end)^2;
    Jtotal2(cnt) = J2(end) + 0.5 * HT * x2(end)^2;
    Jtotal3(cnt) = J3(end) + 0.5 * HT * x3(end)^2;

    %%%%%%% Plotting

    figure(1);

    subplot(2,1,1);
    plot(t,x1,'b-','LineWidth',lw);
    plot(t,x2,'r--','LineWidth',lw);
    plot(t,x3,'g-.','LineWidth',lw);

    subplot(2,1,2);
    plot(t,u1,'b-','LineWidth',lw);
    plot(t,u2,'r--','LineWidth',lw);
    plot(t,u3,'g-.','LineWidth',lw);

end

fprintf( "Avg. Jtotal1 (from analytically computed u) = %2.10f\n", sum(Jtotal1) / N );
fprintf( "Avg. Jtotal2 (from input extracted from Pengcheng's code) = %2.10f\n", sum(Jtotal2) / N );
fprintf( "Avg. Jtotal3 (from u = -grad(v)) = %2.10f\n", sum(Jtotal3) / N );
fprintf( "Value function @x0 = %2.10f\n", double( subs(v, [tvar;xvar], [0;x0]) ) );