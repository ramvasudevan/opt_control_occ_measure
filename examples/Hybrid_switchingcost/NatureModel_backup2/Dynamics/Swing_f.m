function [out] = Swing_f( in1, params )

if nargin < 2
    m = 1;
    g = 9.8;
    l0 = 1;
else
    m = params.m;
    g = params.g;
    l0 = params.l0;
end

myq1    = in1(1,:);
mydq1   = in1(2,:);
myq2    = in1(3,:);
mydq2   = in1(4,:);

out = ...
  [mydq1;
  m.^(-1).*(0.E-323+(-1).*mydq2.*(0.E-323+(-0.1E1).*m.* ...
  mydq2.*myq1)+(-1).*g.*m.*cos(myq2));
  [mydq2];
  m.^(-1).*myq1.^(-2).*( ...
  (-1).*mydq2.*(0.E-323+0.1E1.*m.*mydq1.*myq1)+(-1).*mydq1.*( ...
  0.E-323+0.1E1.*m.*mydq2.*myq1)+g.*m.*myq1.*sin(myq2))];