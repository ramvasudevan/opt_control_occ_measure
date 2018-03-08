
function  LegParts=DrawLegs(vecS,l_leg, gamma_leg)

% figure;
% axis equal
% box on 
% grid on
% vecS  = [0,0]';
% l_leg = 1;
% gamma_leg = 0;

% vecS  = [1,1]';
% l_leg = 0.8;
% gamma_leg = 0.5;

[LegVertices, LegFaces] = ComputeLegGraphics(vecS,l_leg,gamma_leg);
% Spring Part 1************************************************************
L1   = patch('faces', LegFaces.L_Sp1, 'vertices', LegVertices.L_Sp1,...
       'linewidth',4,'EdgeColor',[0 68 158]/256); % color blue
   
% Upper Leg****************************************************************
% 1. outline of upper leg
L2_1  = patch('faces',LegFaces.L_UpBO, 'vertices', LegVertices.L_UpBO,...
      'LineStyle','none','FaceColor',[1 1 1]);
% 2. Draw shaded region in the upper leg
L3   = patch('faces', LegFaces.L_Ups,  'vertices', LegVertices.L_Ups,...
      'linewidth',3,'FaceColor','none','EdgeColor',0.8*[1 1 1]); % grey
% 3. Outline of upper leg  
L2_2  = patch('faces',LegFaces.L_UpBO, 'vertices', LegVertices.L_UpBO,...
      'linewidth',3,'FaceColor','none');

% Lower Leg ***************************************************************
L4  = patch('faces', LegFaces.L_low, 'vertices', LegVertices.L_low,...
      'linewidth',3,'FaceColor',[1 1 1],'EdgeColor',[0 0 0]);

% Spring Part 2 ***********************************************************
L5   = patch('faces',LegFaces.L_Sp2, 'vertices', LegVertices.L_Sp1,...
      'linewidth',4,'EdgeColor',[0 68 158]/256);

% Save all the handles for update function
LegParts = struct('L_Sp1',L1,'L_UpB',L2_1,'L_Upo',L2_2,'L_Ups',L3,'L_low',L4,'L_Sp2',L5);


end
