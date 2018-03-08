% Event function for pronking gait simulator
function [position,isterminal,direction] = PronkingGaitEventsFcn(~,y)
y = y(2:end);

% disp(y(3));
position = [ Guard_L( y )       % Left leg touch-down
             Guard_L( y )       % Left leg lift-off
             Guard_R( y )       % Right leg touch-down
             Guard_R( y )       % Right leg lift-off
             y(3) ];            % Apex
isterminal = ones( 5, 1 );
direction = [ -1; 1; -1; 1; -1 ];