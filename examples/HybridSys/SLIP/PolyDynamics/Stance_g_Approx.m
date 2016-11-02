function [out] = Stance_g_Approx(~)
params = SLIPParams;
k = params.k;
m = params.m;
umax = params.umax;

out = [ 0;
        k/m * umax / 2;
        0;
        0;
        0 ];