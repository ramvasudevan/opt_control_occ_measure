%%%%%%% dx = ( a * x + b * u ) dt + sig * dw

a = -1;
b = 1;
sig = 0.1;

%%%%%%% J = E[ \int_t^T (0.5 * hx * x^2 + 0.5 * r * u^2 ) ds + 0.5 * HT * x(T)^2 ] 

hx = 0;  
r = 1;   
HT = 1;

%%%%%%% Initial Conditions

t0 = 0;
T  = 1;
x0 = 0.5;


%%%%%%% Wiener Process and Time

NW = 1000;
dt = T/NW;
t = [t0 :dt:T]';
dW = randn(1,NW)*sqrt(dt);

%%%%%%% Analytically Optimal Solution: u^o = 2*x / ( 3 * exp(2*(1-t)) - 1 )

i = 1;
x1(i) = x0;
u1(i)= 2*x1(i) / ( 3 * exp(2*(1-t(i))) - 1 );
J1(i) = 0 + dt * (0.5 * hx * x1(i)^2 + 0.5 * r * u1(i)^2);

for i = 2:NW+1
    x1(i) = x1(i-1) + (a * x1(i-1) + b * u1(i-1)) * dt + sig * dW(i-1);
    u1(i)= 2*x1(i) / ( 3 * exp(2*(1-t(i))) - 1 );
    J1(i) = J1(i-1) + dt * (0.5 * hx * x1(i)^2 + 0.5 * r * u1(i)^2);
end

Jtotal = J1(end) + 0.5 * HT * x1(end)^2

%%%%%%% Plotting

lw = 2;
ftsz = 18;

subplot(2,1,1)
hold all
grid on
box on
plot(t,x1,'b','LineWidth',lw)
ylabel('x','FontSize',ftsz)

subplot(2,1,2)
hold all
grid on
box on
plot(t,u1,'b','LineWidth',lw)
ylabel('u','FontSize',ftsz)