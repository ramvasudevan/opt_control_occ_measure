% Generate figures for SLIP

clear;
close all;

figure(1);
subplot(4,1,4);

%% Plot GPOPS result
% load('data/gpops_constV2_T4');
% load('data/gpops_V2_nphases9');
load('data0519/gpops_Vd2_T2d8_nphase9');

mode_seq = mod( (1:nphases)' + 1, 3 ) + 1;
state_hist_std = [];
mode_hist = [];
t_hist = [];
for i = 1 : nphases
    s = output.result.solution.phase(i).state;
    t_hist = [ t_hist; output.result.solution.phase(i).time ];
    len = size(s,1);
    switch mode_seq(i)
        case 1      % Stance phase
            state_hist_std = [ state_hist_std;
                               s, nan*ones(len,1), s(:,1) .* cos(s(:,3)), nan*ones(len,1)];
            mode_hist = [ mode_hist; ones(len,1) * 1 ];
        otherwise
            state_hist_std = [ state_hist_std;
                               nan*ones(len,4), s];
            mode_hist = [ mode_hist; ones(len,1) * mode_seq(i) ];
    end
end
state_hist_std = [ state_hist_std, mode_hist ];
x_hist_std = state_hist_std( :, 5 );
y_hist_std = state_hist_std( :, 7 );

state_hist = state_hist_std;
x_hist = x_hist_std;
y_hist = y_hist_std;
mycolor = [ 0 0 1 ];
PlotFrame_helper;

%% Plot our results
subplot(4,1,1);
disp('Processing degree 4');
load('data0519/constV_Vd2_T2d8_d4');
t_hist = t_hist * T;
mycolor = [.988,.553,.349];
PlotFrame_helper;

subplot(4,1,2);
disp('Processing degree 6');
load('data0519/constV_Vd2_T2d8_d6');
t_hist = t_hist * T;
mycolor = [.890,.290,.2];
PlotFrame_helper;

subplot(4,1,3);
disp('Processing degree 8');
load('data0519/constV_Vd2_T2d8_d8');
t_hist = t_hist * T;
mycolor = [.702,0,0];
PlotFrame_helper;