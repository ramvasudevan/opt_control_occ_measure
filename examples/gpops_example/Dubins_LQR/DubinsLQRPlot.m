%-------------------------------------------------------------------------%
%                             Extract Solution                            %
%-------------------------------------------------------------------------%

solution = output.result.solution;
time1 = solution.phase(1).time;
state1 = solution.phase(1).state;
control1 = solution.phase(1).control;

time2 = solution.phase(2).time;
state2 = solution.phase(2).state;
control2 = solution.phase(2).control;

time3 = solution.phase(3).time;
state3 = solution.phase(3).state;
control3 = solution.phase(3).control;

time = [ time1; time2; time3 ];
state = [ state1; state2; state3 ];
control = [ control1; control2; control3 ];

figure(1);
hold on;
plot(state(:,1), state(:,2),'LineWidth',1);

figure(2);
hold on;
plot(time, control(:,1));
