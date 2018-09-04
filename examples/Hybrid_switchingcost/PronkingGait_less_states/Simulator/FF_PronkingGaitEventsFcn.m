% Event function for pronking gait simulator
function [position,isterminal,direction] = FF_PronkingGaitEventsFcn(~,y)
% y = y(2:end);
y = y(2:end-1);
% disp(y(3));
position = [ Guard_L( y )       % Left leg touch-down
             nan                % Left leg lift-off
             Guard_R( y )       % Right leg touch-down
             nan                % Right leg lift-off
             nan ];            % Apex
isterminal = ones( 5, 1 );
direction = [ -1; 1; -1; 1; -1 ];