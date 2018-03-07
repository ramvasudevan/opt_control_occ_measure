function [ LegLength, LegAngle, BodyJPos, BackJPos, FrontJPos] = ComputeJoint_LegLA(y,tEvents,T)

% Get a mapping for the state and parameter vectors.  This allows us
% to use a more readable syntax: "y(contStateIndices.dy)" instead of
% "y(3)" while still operating with vectors and not with structs.
% We keep the index-structs in memory to speed up processing
% persistent contStateIndices  systParamIndices discStateIndices
% if isempty(contStateIndices) || isempty(discStateIndices) || isempty(systParamIndices) 
%     [~, ~, contStateIndices] = ContStateDefinition();
%     [~, ~, discStateIndices] = DiscStateDefinition();
%     [~, ~, systParamIndices] = SystParamDefinition();
% end

la       = 0.5; % half of the body length

l_l      = 1;
l_r      = 1;


tB_TD = tEvents(1);
tB_LO = tEvents(2);
tF_TD = tEvents(3);
tF_LO = tEvents(4);
tAPEX = tEvents(5);
% tAPEX = P(5);

    if tB_TD < 0
         tB_TD = tB_TD + tAPEX;
    end
    if tB_TD > tAPEX
         tB_TD = tB_TD - tAPEX;
    end 
    
    if tB_LO < 0
         tB_LO = tB_LO + tAPEX;
    end
    if tB_LO > tAPEX
         tB_LO = tB_LO - tAPEX;
    end 
    
    if tF_LO < 0
         tF_LO = tF_LO + tAPEX;
    end
    if tF_LO > tAPEX
         tF_LO = tF_LO - tAPEX;
    end
    
    if tF_TD < 0
         tF_TD = tF_TD + tAPEX;
    end
    if tF_TD > tAPEX
         tF_TD = tF_TD - tAPEX;
    end
    
    
if ((T>tB_TD && T<tB_LO && tB_TD<tB_LO) || ((T<tB_LO || T>tB_TD) && tB_TD>tB_LO))
    contactB = true;
else
    contactB = false;
end
if ((T>tF_TD && T<tF_LO && tF_TD<tF_LO) || ((T<tF_LO || T>tF_TD) && tF_TD>tF_LO))
    contactF = true;
else
    contactF = false;
end


yx       = y(1);
yy       = y(3);
phi      = y(5);
alphaB   = y(7);
alphaF   = y(9);


x_b = -la*cos(phi) + yx;
x_f =  la*cos(phi) + yx;
y_b = -la*sin(phi) + yy;
y_f =  la*sin(phi) + yy;


if contactB == true
    l_leg_B = y_b/cos(alphaB+phi);    
else    
    l_leg_B     = l_l;
end

gamma_leg_B = alphaB + phi; 

if contactF == true
    l_leg_F = y_f/cos(alphaF+phi);    
else    
    l_leg_F = l_r;
end

gamma_leg_F = alphaF + phi; 

                        


LegLength = struct('B',l_leg_B,'F',l_leg_F);
LegAngle  = struct('B',gamma_leg_B,'F',gamma_leg_F);
BodyJPos  = [yx, yy, phi];
BackJPos  = [x_b;y_b];
FrontJPos = [x_f;y_f];

end

