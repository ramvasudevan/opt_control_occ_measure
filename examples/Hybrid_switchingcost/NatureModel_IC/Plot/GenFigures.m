% Generate figures
clear;
close all;


h_fig1 = figure(3);
subplot(3,1,1);
plotdata_freeIC(gca, 'Result_Fpoly3_T3_noIC_deg6', [0.35; 0; 0; 0.7257], [.988,.553,.349]);

subplot(3,1,2);
plotdata_freeIC(gca, 'Result_Fpoly3_T3_noIC_deg8', [0.35; 0; 0; 0.73097], [.890,.290,.2]);

subplot(3,1,3);
plotdata_freeIC_gpops( gca, 'Result_Gpops_T3_noIC' );

% print(h_fig1,'Example1','-dpdf');

h_fig2 = figure(4);
subplot(3,1,1);
plotdata_freeIC(gca, 'Result_Fpoly3_T7_noIC_deg6', [0.35; 0; 0; 0.7], [.988,.553,.349]);

subplot(3,1,2);
plotdata_freeIC(gca, 'Result_Fpoly3_T7_noIC_deg8', [0.35; 0; 0; 0.7], [.890,.290,.2]);

subplot(3,1,3);
plotdata_freeIC_gpops( gca, 'Result_Gpops_T7_noIC' );
% print(h_fig2,'Example2','-dpdf');
