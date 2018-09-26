% Generate figures
clear;
close all;


% h_fig1 = figure(3);
% subplot(3,1,1);
% plotdata_fixedIC(gca, 'Result_Fpoly3_T3_fixedIC_deg6', [ 0.35; 0; 0; 0.85 ], [.988,.553,.349]);
% 
% subplot(3,1,2);
% plotdata_fixedIC(gca, 'Result_Fpoly3_T3_fixedIC_deg8', [ 0.35; 0; 0; 0.85 ], [.890,.290,.2]);
% 
% subplot(3,1,3);
% plotdata_gpops( gca, 'Result_Gpops_T3_fixedIC' );
% 
% print(h_fig1,'Example1','-dpdf');

h_fig2 = figure(4);
subplot(3,1,1);
plotdata_fixedIC(gca, 'Result_Fpoly3_T8_fixedIC_deg6', [ 0.35; 0; 0; 0.85 ], [.988,.553,.349]);

subplot(3,1,2);
plotdata_fixedIC(gca, 'Result_Fpoly3_T8_fixedIC_deg8', [ 0.35; 0; 0; 0.85 ], [.890,.290,.2]);

subplot(3,1,3);
plotdata_gpops( gca, 'Result_Gpops_T7_fixedIC_v2' );
print(h_fig2,'Example2','-dpdf');
