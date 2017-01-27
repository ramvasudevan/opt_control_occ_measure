%-------------------------------------------------------------------------%
%                             Extract Solution                            %
%-------------------------------------------------------------------------%

pval = output.result.objective;
solution = output.result.solution;

tval = [];
x_state = [];
y_state = [];
control = [];

offset = output.result.setup.auxdata.init;

for iphase = 1 : nphases
    idx = mod(iphase+offset,3) + 1;
    tval = [ tval; solution.phase(iphase).time ];
    switch idx
        case 1
            tmp = solution.phase(iphase).state;
            x_state = [ x_state; tmp(:,5) ];
            y_state = [ y_state; tmp(:,1).*cos(tmp(:,3)) ];
            control = [ control; solution.phase(iphase).control ];
        case {2,3}
            len = size( solution.phase(iphase).state, 1 );
            x_state = [ x_state; solution.phase(iphase).state(:,1) ];
            y_state = [ y_state; solution.phase(iphase).state(:,3) ];
            control = [ control; -2 * ones(len,1) ];
    end
    
end


figure(1);
hold on;
if exist('mycolor','var') && exist('mythickness','var')
    plot(x_state, y_state, 'color', mycolor, 'LineWidth', mythickness);
else
    plot(x_state, y_state)
end

figure(2);
hold on;
plot(tval, control)

% figure(3);
% hold on;
% plot(tval, y_state);