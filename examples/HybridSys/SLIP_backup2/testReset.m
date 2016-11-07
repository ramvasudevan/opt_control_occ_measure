% Evaluate the reset map at different points

params = SLIPParams;

[X1, X2, X3, X4, X5] = ndgrid( linspace( 0.5, 1, 15 ), ...
                              linspace( 0, 5, 15 ), ...
                              linspace(-pi/3, pi/3, 15 ), ...
                              linspace(-2*pi/3, 0, 15 ), ...
                              linspace(0, 5, 15) );
xval = [X1(:), X2(:), X3(:), X4(:), X5(:)];
data1 = zeros(size(xval,1), 5);
for i = 1 : size(xval,1)
    data1(i,:) = Reset_simp(xval(i,:),params);
end

max(abs(data1))


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
                               linspace( 0, 1, 10 ), ...
                               linspace( -1, 1, 10 ), ...
                               linspace( -1, 1, 10 ), ...
                               linspace( -1, 1, 10 ) );
xval = [X1(:), X2(:), X3(:), X4(:), X5(:)];
data1 = zeros(size(xval,1), 5);
for i = 1 : size(xval,1)
    data1(i,:) = rescale_reset(@(x) Reset_simp(x,params), xval(i,:)', scale_x, trans_x, 1, 0 );
%     data1(i,:) = rescale_reset(@(x) Reset_simp(x,params), xval(i,:)', scale_x, trans_x, scale_x, trans_x );
end

max(abs(data1))