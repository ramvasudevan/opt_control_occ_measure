clear all,close all,clc;
load temp
[ moment_vec,moment_vec_based,moment_mat,moment_mat_based,Adim] = Pre_process_linear( 2,6 );
[x,p,~] = decomp(temp(:,1));
val = double(temp(:,2));
p = full(p)*[7;1];
moment = 0*moment_mat;
for i = 1:length(p),
    moment = moment + (moment_mat==p(i))*val(i);
end
