function  BodyH = DrawBody(BodyJPos)
% Create patches of main body in the Constructor. Save handles for update
% function.

% figure;
% axis equal
% box on 
% grid on
% % BodyJPos = [0,0,0];
% BodyJPos = [1,1,0.5];

[ x1, y1, f,v ]  = ComputeBodyGraphics(BodyJPos);

% White background (face color white, no line)
b1  = patch('XData',x1,'YData',y1,'LineStyle','none','FaceColor',[1 1 1]); 
% Shade lines (Grey lines)
b2  = patch('faces', f, 'vertices', v,...
       'linewidth',3,'FaceColor',[1 1 1],'EdgeColor',0.8*[1 1 1]); 
% Black Outline (No face color white, black outline)
b3  = patch('XData',x1,'YData',y1,'linewidth',4,'FaceColor','none');
         
% Save all the handles for update function
BodyH = struct('B_bg',b1,'B_sha',b2,'B_out',b3);

end