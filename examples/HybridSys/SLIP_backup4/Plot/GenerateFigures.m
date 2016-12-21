% Generate a figure with several subplots
close all;
clear;


%% Get far!!
figure;

h = subplot(4,1,1);
hold on;
plotdata('', h, 4);

h = subplot(4,1,2);
hold on;
plotdata('data/Far_T6_d6_dist4', h, 4);

h = subplot(4,1,3);
hold on;
plotdata('data/Far_T6_d6_dist6', h, 6);

h = subplot(4,1,4);
hold on;
plotdata('data/Far_T6_d6_dist8', h, 8);

%% Get far, and save energy


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


