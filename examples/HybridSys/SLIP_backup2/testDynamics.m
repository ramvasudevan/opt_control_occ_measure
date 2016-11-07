% % Evaluate the dynamics at different points;
% 
params = SLIPParams;

% Stance
[X1, X2, X3, X4, X5] = ndgrid( linspace( 0.5, 1, 10 ), ...
                              linspace( -5, 5, 10 ), ...
                              linspace(-pi/3, pi/3, 10 ), ...
                              linspace(-2*pi/3, 0, 10 ), ...
                              linspace(0, 5, 10) );
xval = [X1(:), X2(:), X3(:), X4(:), X5(:)];
data1 = zeros(size(xval,1), 5);
for i = 1 : size(xval,1)
    data1(i,:) = Stance_simp_f(xval(i,:),params);
end

max(abs(data1))
% 
% % Flight
% [X1, X2, X3, X4] = ndgrid( linspace( 0, 5, 15 ), ...
%                            linspace( 0, 3, 15 ), ...
%                            linspace(0.5, 2, 15 ), ...
%                            linspace(-6, 6, 15 ) );
% xval = [X1(:), X2(:), X3(:), X4(:)];
% data2 = zeros(size(xval,1), 4);
% for i = 1 : size(xval,1)
%     data2(i,:) = Flight_Approx_f(xval(i,:),params);
% end
% 
% max(abs(data2))


%% Scaled dynamics
domain_size = [ 0.5, 1;
                -5, 5;
                -pi/3, pi/3;
                -2*pi/3, 0;
                0, 5 ];

for i = 1 : 1
    scale_x = (domain_size(:,2) - domain_size(:,1)) / 2;
    trans_x = mean(domain_size,2);
end
[X1, X2, X3, X4, X5] = ndgrid( linspace( -1, 1, 10 ), ...
                               linspace( -1, 1, 10 ), ...
                               linspace( -1, 1, 10 ), ...
                               linspace( -1, 1, 10 ), ...
                               linspace( -1, 1, 10 ) );
xval = [X1(:), X2(:), X3(:), X4(:), X5(:)];
data1 = zeros(size(xval,1), 5);
for i = 1 : size(xval,1)
    data1(i,:) = rescale_dynamics(@(x) Stance_simp_f(x,params), xval(i,:)', scale_x, trans_x, scale_x );
end

max(abs(data1))
