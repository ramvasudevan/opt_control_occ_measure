% Event function for pronking gait simulator (polynomial version)
function [position,isterminal,direction] = LF_PronkingGaitEventsFcn_poly(~,y)

y = y(2:end);

t1 = Guard_L_poly( y );
t2 = Guard_R_poly( y );

position = [ nan        % Left leg touch-down
             t1(1)      % Left leg lift-off
             t2(1)      % Right leg touch-down
             nan        % Right leg lift-off
             nan ];     % Apex
isterminal = ones( 5, 1 );
direction = [ -1; 1; -1; 1; -1 ];