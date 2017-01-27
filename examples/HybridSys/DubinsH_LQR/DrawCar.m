function DrawCar(state)

% Compute end points
a = 0.1;
b = 0.05;

xpos = state(1);
ypos = state(2);
theta = state(3);

M = [cos(theta), -sin(theta);
     sin(theta), cos(theta)];
pts = repmat([ xpos; ypos ],1,4) + M * ...
      [ a, -a, -a, a;
        0, -b, b, 0 ];

plot(pts(1,:), pts(2,:), 'k');