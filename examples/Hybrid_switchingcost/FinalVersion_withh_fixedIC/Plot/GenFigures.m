% Generate figures
clear;
close all;


h_fig1 = figure(3);
subplot(4,1,1);
plotdata_fixedIC(gca, 'Result_Fpoly3_T3_fixedIC_deg4', [ 0.35; 0; 0; 0.85 ], [.988,.553,.349]);

subplot(4,1,2);
plotdata_fixedIC(gca, 'Result_Fpoly3_T3_fixedIC_deg6', [ 0.35; 0; 0; 0.85 ], [.890,.290,.2]);

subplot(4,1,3);
plotdata_fixedIC(gca, 'Result_Fpoly3_T3_fixedIC_deg8', [ 0.35; 0; 0; 0.85 ], [.702,0,0]);

subplot(4,1,4);
plotdata_gpops( gca, 'Result_Gpops_T3_fixedIC' );

set(h_fig1,'PaperSize',[6, 7.4]);
h_fig1.PaperUnits = 'inches';
h_fig1.PaperPosition = [-0.4, -0.6, 6.8 8.4];
print(h_fig1,'Example1_fixedIC','-dpdf');

% ============================================================================================
h_fig2 = figure(4);
subplot(4,1,1);
plotdata_fixedIC(gca, 'Result_Fpoly3_T7_fixedIC_deg4', [ 0.35; 0; 0; 0.85 ], [.988,.553,.349]);

subplot(4,1,2);
plotdata_fixedIC(gca, 'Result_Fpoly3_T7_fixedIC_deg6', [ 0.35; 0; 0; 0.85 ], [.890,.290,.2]);

subplot(4,1,3);
plotdata_fixedIC(gca, 'Result_Fpoly3_T7_fixedIC_deg8', [ 0.35; 0; 0; 0.85 ], [.702,0,0]);

subplot(4,1,4);
plotdata_gpops( gca, 'Result_Gpops_T7_fixedIC' );

set(h_fig2,'PaperSize',[6, 7.4]);
h_fig2.PaperUnits = 'inches';
h_fig2.PaperPosition = [-0.4, -0.6, 6.8 8.4];
print(h_fig2,'Example2_fixedIC','-dpdf');


%% 
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
% 
% h_fig2 = figure(4);
% subplot(3,1,1);
% plotdata_fixedIC(gca, 'Result_Fpoly3_T7_fixedIC_deg6', [ 0.35; 0; 0; 0.85 ], [.988,.553,.349]);
% 
% subplot(3,1,2);
% plotdata_fixedIC(gca, 'Result_Fpoly3_T7_fixedIC_deg8', [ 0.35; 0; 0; 0.85 ], [.890,.290,.2]);
% 
% subplot(3,1,3);
% plotdata_gpops( gca, 'Result_Gpops_T7_fixedIC' );
% print(h_fig2,'Example2','-dpdf');