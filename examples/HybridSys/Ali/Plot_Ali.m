load('HalfCricletf5w200sw200circ.mat');

nk = 200;
nts = 200;

figure(1)
hold on

for j = nts:-1:1
   for k = 1 : nk
       x10(j,k) = x1(1,k,1,j);
       x20(j,k) = x1(2,k,1,j);
       v01(j,k) = .5 * x1(:,k,1,j)'*Pi1(:,:,1,j)*x1(:,k,1,j) + s1(:,k,1,j)'*x1(:,k,1,j)+alpha(k,1,j);
       plot3(x10(j,k),x20(j,k),v01(j,k),'b.');
   end
end
xlim([-1,1]);
ylim([-1,1]);
zlim([-1,2]);