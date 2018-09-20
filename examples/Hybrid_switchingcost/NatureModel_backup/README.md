Fixed IC.
h = u^2
c = (d - d_dex)^2
m = 0.2
g = 0.2
l0 = 0.5
lmax = 1

Trajectories seem to be too close to the boundary (i.e. on the verge of falling).
Result_T4_fixedIC_deg8.mat is the only case that worked well for us.
Gpops isn't working well enough.

the optimal u is roughly in [0,0.17]. So I changed m to be 1 in newer versions of my code.

May need to further modify the model.
