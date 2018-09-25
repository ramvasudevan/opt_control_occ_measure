% Time gpops
clc;
t0 = tic;
for nphases = 2 : 10
    try
        TimeGpops_helper( nphases, 3 );
    end
end
t1 = toc(t0);

t0 = tic;
for nphases = 2 : 10
    try
        TimeGpops_helper( nphases, 7 );
    end
end
t2 = toc(t0);