function [out] = Reset_S2S_poly3_revised( in1, params )

if nargin < 2
    m = 1;
    g = 9.8;
    l0 = 1;
    alpha = pi/6;
    error('No params!');
else
    m = params.m;
    g = params.g;
    l0 = params.l0;
    alpha = params.alpha;
end

myq1    = in1(1,:);     % l
mydq1   = in1(2,:);     % l_dot
myq2    = in1(3,:);     % theta
mydq2   = in1(4,:);     % theta_dot

out = ...
  [l0;
  (12+(-6).*alpha+alpha.^3).^(-1).*(6.*mydq2.*myq2.*(alpha.* ...
  myq1+(-1).*l0.*myq2)+3.*mydq1.*(4+alpha.*((-2)+myq2.^2)));
  alpha;
  alpha.^(-1).*(12+(-6).*alpha+alpha.^3).^(-1).*l0.^(-1).*(6.*myq2.* ...
  (2.*mydq2.*myq1+mydq1.*myq2+(-1).*l0.*mydq2.*myq2)+alpha.^2.*((-6) ...
  .*mydq1+3.*l0.*mydq2.*myq2.^2)+(-1).*alpha.^3.*(2.*mydq2.*myq1.* ...
  myq2+mydq1.*((-2)+myq2.^2)))];