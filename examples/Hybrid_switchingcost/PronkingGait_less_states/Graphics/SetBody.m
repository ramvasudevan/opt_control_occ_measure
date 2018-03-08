function SetBody(BodyJPos, Body)
% Set the patches of main body when figure is updated.

[ x1, y1, f,v ] = ComputeBodyGraphics(BodyJPos);

set(Body.B_bg , 'xData', x1, 'yData',    y1);                   
set(Body.B_sha, 'faces',  f, 'vertices',  v);
set(Body.B_out, 'xData', x1, 'yData',    y1);   

end