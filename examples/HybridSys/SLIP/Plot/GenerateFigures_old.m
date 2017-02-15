% Generate a figure with several subplots

%% V1
close all;
clear;

load('data/gpops_constV1_T4');
mycolor = [0 0 1];
mythickness = 8;
PlotGpopsTrajectory;

load('data/constV1_d4_T4');
mycolor = [.702,0,0];
mythickness = 6;
PlotRelaxedControl;

load('data/constV1_d6_T4');
mycolor = [.890,.290,.2];
mythickness = 4;
PlotRelaxedControl;

load('data/constV1_d8_T4');
mycolor = [.988,.553,.349];
mythickness = 2;
PlotRelaxedControl;

xlim([-1,1]);
ylim([-1,1]);

%% V2
close all;
clear;

load('data/constV2_d4_T4');
mycolor = [.702,0,0];
mythickness = 6;
PlotRelaxedControl;

load('data/constV2_d6_T4');
mycolor = [.890,.290,.2];
mythickness = 4;
PlotRelaxedControl;

% load('data/constV2_d8_T4');
% mycolor = [.988,.553,.349];
% mythickness = 2;
% PlotRelaxedControl;


%% Get far, save energy, and jump high in the end
figure;

h = subplot(4,1,1);
hold on;
plotdata('', h, 4);

h = subplot(4,1,2);
hold on;
plotdata('data/FarHighEnergy_T6_d6_dist4', h, 4);

h = subplot(4,1,3);
hold on;
plotdata('data/FarHighEnergy_T6_d6_dist6', h, 6);

h = subplot(4,1,4);
hold on;
plotdata('data/FarHighEnergy_T6_d6_dist8', h, 8);


