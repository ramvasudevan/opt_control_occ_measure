
function  SetLegs(vecS,l_leg, gamma_leg,Leghandle)

[LegVertices, LegFaces] = ComputeLegGraphics(vecS,l_leg,gamma_leg);
% Spring Part 1 (Zigzag line)**********************************************
set(Leghandle.L_Sp1,'faces', LegFaces.L_Sp1,  'vertices', LegVertices.L_Sp1);
% Upper Leg****************************************************************
% 1. Background of upper leg
set(Leghandle.L_UpB,'faces', LegFaces.L_UpBO, 'vertices', LegVertices.L_UpBO );
% 2. Shade the upper leg
set(Leghandle.L_Ups,'faces', LegFaces.L_Ups,  'vertices', LegVertices.L_Ups);
% 3. Outline the upper leg
set(Leghandle.L_Upo,'faces', LegFaces.L_UpBO, 'vertices', LegVertices.L_UpBO );
% Lower Leg ***************************************************************
set(Leghandle.L_low,'faces', LegFaces.L_low,  'vertices', LegVertices.L_low );
% Spring Part 2 (parallel)*************************************************
set(Leghandle.L_Sp2,'faces', LegFaces.L_Sp2,  'vertices', LegVertices.L_Sp1);

end
