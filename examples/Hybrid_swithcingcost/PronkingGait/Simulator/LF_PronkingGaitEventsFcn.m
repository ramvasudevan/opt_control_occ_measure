% Event function for pronking gait simulator
function [position,isterminal,direction] = LF_PronkingGaitEventsFcn(~,y)
y = y(2:end);

% disp(y(3));
position = [ nan                % Left leg touch-down
             Guard_L( y )       % Left leg lift-off
             Guard_R( y )       % Right leg touch-down
             nan                % Right leg lift-off
             nan ];             % Apex
isterminal = ones( 5, 1 );
direction = [ -1; 1; -1; 1; -1 ];