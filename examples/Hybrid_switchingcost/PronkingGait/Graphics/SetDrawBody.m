function SetDrawBody(vecS, gamma, Body)

[ x1, y1, f,v ] = ComputeBodyGraphics( vecS, gamma);

set(Body.B_bg ,'xData',x1,'yData', y1);                   
set(Body.B_sha,'faces', f, 'vertices', v);
set(Body.B_out,'xData',x1,'yData', y1);   

end