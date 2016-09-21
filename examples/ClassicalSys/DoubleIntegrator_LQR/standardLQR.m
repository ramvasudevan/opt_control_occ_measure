% clear;

A = [ 0, 1; 0, 0 ];
B = [ 0; 1 ];
Q = eye(2);
R = 20;
F = 0 * eye(2);

tSpan = [ 0, 5 ];
nSoln = 501;
tol = 1e-5;

x0 = [1;1];

sol = finiteLqr( tSpan,A,B,Q,R,F,nSoln,tol );
tspan = linspace( 0, 5, 501 );

[tval, xval] = ode45( @(t,x) DI_LinFdbk(t,x,sol, Q, R), tspan, [x0;0] );

cost = xval(:,end);
xval = xval(:,1:2);

uval = zeros(size(tval));
for i = 1 : length(tval)
    uval(i) = - sol(i).K * xval(i,:)';
end

% subplot(1,2,1);
plot(xval(:,1), xval(:,2));
hold on;
plot(x0(1), x0(2), 'ro');
plot(0,0,'rx');
xlim([0,2]);
ylim([-1,1]);
save('LQRdata');

% control
figure;
uval = 0 * tval;
for i = 1 : length(tval)
    idx = ceil( tval(i) * 100 + 1e-5 );
    uval(i) = - sol(idx).K * xval(i,:)';
end
plot(tval,uval);