function [ out ] = PolyCombine( p1, p2 )

l1 = length( p1 );
l2 = length( p2 );

out = msspoly( zeros( l1*l2, 1 ) );
cnt = 1;

for i = 1 : l1
    for j = 1 : l2
        out(cnt) = p1(i) * p2(j);
        cnt = cnt + 1;
    end
end
