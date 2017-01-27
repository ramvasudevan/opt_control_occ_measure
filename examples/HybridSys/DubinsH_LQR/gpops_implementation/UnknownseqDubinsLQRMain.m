% Dubins car model, minimum time problem, unknown transition sequence
% 1 | 2
% --|--
% 3 | 4
% 
% state = (x,y,theta)
% control = (V,omega)
% 

clear;

% -------------------------------------
%     Define domains of each mode
% -------------------------------------
domain = cell(4,1);
domain{1} = [ -1, 0;
               0, 1;
              -pi/2, pi/2];
domain{2} = [ 0, 1;
              0, 1;
             -pi/2, pi/2 ];
domain{3} = [ -1, 0;
              -1, 0;
              -pi/2, pi/2];
domain{4} = [ 0, 1;
             -1, 0;
             -pi/2, pi/2];

guess_helper = cell(4,4);
guess_helper{1,2} = [ 0, 0.5, 0 ];
guess_helper{1,3} = [ -0.5, 0, -pi/4 ];
guess_helper{2,4} = [ 0.5, 0, -pi/4 ];
guess_helper{3,4} = [ 0, -0.5, 0 ];


% -------------------------------------
%    Search for transition sequences
% -------------------------------------
arr = [ 1, 1, 1;
        1, 1, 2;
        1, 1, 3;
        1, 2, 2;
        1, 2, 4;
        1, 3, 3;
        1, 3, 4 ];

sol = cell(7,1);
x0 = [ -0.8, 0.8, 0 ];

tic;
for i = 1 : 7
    seq = arr(i,:);
    seq = seq([1,diff(seq)]~=0);
    
    sol{i} = DubinsLQR_helper(i, seq, domain, x0, guess_helper);
%     pause;
end
gpops_time = toc;


