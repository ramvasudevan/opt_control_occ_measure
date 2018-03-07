% Event function for pronking gait simulator
function [position,isterminal,direction] = FR_PronkingGaitEventsFcn(~,y)
y = y(2:end);

% disp(y(3));
position = [ Guard_L( y )       % Left leg touch-down
             nan                % Left leg lift-off
             nan                % Right leg touch-down
             Guard_R( y )       % Right leg lift-off
             nan ];             % Apex
isterminal = ones( 5, 1 );
direction = [ -1; 1; -1; 1; -1 ];