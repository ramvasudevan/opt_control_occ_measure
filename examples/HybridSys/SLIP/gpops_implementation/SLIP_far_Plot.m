%-------------------------------------------------------------------------%
%                             Extract Solution                            %
%-------------------------------------------------------------------------%

pval = output.result.objective;
solution = output.result.solution;

time1 = solution.phase(1).time;
state1 = solution.phase(1).state;

time2 = solution.phase(2).time;
tmp = solution.phase(2).state;
% polycos = @(xx) sqrt(3)/2 + 0.5*(xx+pi/6);
yval = tmp(:,1).*cos(tmp(:,3));
state2 = [ solution.phase(2).state(:,5), yval ];
control2 = solution.phase(2).control;

time3 = solution.phase(3).time;
state3 = solution.phase(3).state;

tval = [ time1; time2; time3 ];
x_state = [ state1(:,1); state2(:,1); state3(:,1) ];
y_state = [ state1(:,3); state2(:,2); state3(:,3) ];

figure(1);
hold on;
plot(x_state, y_state);