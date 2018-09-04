function  ShowTrajectory( T,Y,P )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

PlotFlag = 1;
RecordVideo = 0;
plotGRF = 0;
plotAnimation = 1;
saveFig = 0;
RecordGIF = 0;
PlotLC  = 0;
% addpath('D:\Google Drive\Frameworks\ContFrameQuadruped - Apex');
% addpath('D:\Google Drive\Frameworks\ContFrameQuadruped - Apex\Graphics');
pathname = fileparts('D:');
filename = '334.gif';


if PlotLC == 1
   
    figure(1);  hold on;
    plot3(Y(:,2),Y(:,9),Y(:,3),'r-');
    
end    

if PlotFlag
    %% Plot trajectories
%     figure(1); clf; grid on; plot(T,Y,'-');
%     legend('x','dx','y','dy','phi','dphi','alphaB','dalphaB','alphaF','dalphaR');
    % P = [tL_TD,tL_LO,tR_TD,tR_LO,0,0,k,omega];

    tB_TD = P(1);
    tB_LO = P(2);
    tF_TD = P(3);
    tF_LO = P(4);
    tAPEX = P(5);
    k = P(8);
    la = 0.5;

    %% Compute GRFs
    if tB_TD < 0
         tB_TD = tB_TD + tAPEX;
    end
    if tB_TD > tAPEX
         tB_TD = tB_TD - tAPEX;
    end  
    if tB_LO < 0
         tB_LO = tB_LO + tAPEX;
    end
    if tB_LO > tAPEX
         tB_LO = tB_LO - tAPEX;
    end   
    if tF_LO < 0
         tF_LO = tF_LO + tAPEX;
    end
    if tF_LO > tAPEX
         tF_LO = tF_LO - tAPEX;
    end   
    if tF_TD < 0
         tF_TD = tF_TD + tAPEX;
    end
    if tF_TD > tAPEX
         tF_TD = tF_TD - tAPEX;
    end

 if plotGRF   
    n = length(T);
    FBy = zeros(n,1);
    FFy = zeros(n,1);

    for  i = 1:n
        
        

        % Figure out the current contact configuration (this is used in the
        % dynamics function)
        t_ = T(i);
        if ((t_>tB_TD && t_<tB_LO && tB_TD<tB_LO) || ((t_<tB_LO || t_>tB_TD) && tB_TD>tB_LO))
            contactB = true;
        else
            contactB = false;
        end
        if ((t_>tF_TD && t_<tF_LO && tF_TD<tF_LO) || ((t_<tF_LO || t_>tF_TD) && tF_TD>tF_LO))
            contactF = true;
        else
            contactF = false;
        end      

        y_ = Y(i,:);
        x        = y_(1);
        dx       = y_(2);
        y        = y_(3);
        dy       = y_(4);
        phi      = y_(5);
        dphi     = y_(6);            
        alphaB   = y_(7);
        dalphaB  = y_(8);
        alphaF   = y_(9);
        dalphaF  = y_(10);

         pos0 = [x;y];
         posF = pos0 + la*[cos(phi);sin(phi)] ;
         posB = pos0 + la*[cos(phi + pi);sin(phi + pi)] ;
        % Compute forces acting on the main body (only legs in contact
        % contribute): 

        if contactB
            FBy(i) = (1- posB(2)/cos(alphaB+phi))*k*cos(alphaB+phi);
        end
        if contactF
            FFy(i) = (1- posF(2)/cos(alphaF+phi))*k*cos(alphaF+phi);
        end
        
        
        
        
    end

    figure(2); clf; grid on; hold on;
    plot(T,FBy,'color',[127/256,127/256,127/256], 'LineWidth',2);   
    plot(T,FFy,'color',[0/256,45/256,98/256], 'LineWidth',2);  
    legend('Hind','Front');  
 end   
    
    
if plotAnimation
    graphOUTPUT = SLIP_Model_Graphics_4Swing;
         
    tEvents = [tB_TD; tB_LO; tF_TD; tF_LO; tAPEX]; 
    n = round(T(end)*100); % # of frames per step
    tFrame = linspace(0, T(end), n+1);
    frameCount = 1;
    % If desired, every iteration a rendered picture is saved to disc.  This
    % can later be used to create a animation of the monopod.
    for j = 1:n
        y = interp1(T' + linspace(0,1e-5,length(T)), Y, tFrame(j))';
        graphOUTPUT.update(y,tEvents,tFrame(j));
        % (un)comment the following line, if you (don't) want to save the individual frames to disc:
        % fig = gcf;
        % print(fig,'-r200','-djpeg',['MovieFrames/Frame',num2str(frameCount,'%04d.jpg')],'-opengl');
        % print(fig,'-dpdf',['MovieFrames/Frame',num2str(frameCount,'%04d.pdf')]);
        frameCount = frameCount + 1;
    end
end

    if RecordGIF

        graphOUTPUT = SLIP_Model_Graphics_4Swing; % Must be called again with new parameters p, 
        % such that the new angle of attack is visualized
        tEvents = [tB_TD; tB_LO; tF_TD; tF_LO; tAPEX]; 
        n = round(T(end)*50); % # of frames per step
        tFrame = linspace(0, T(end), n+1);

        %get the directory of your input files:
        
        %use that when you save
        matfile = fullfile(pathname, filename);

        % If desired, every iteration a rendered picture is saved to disc.  This
        % can later be used to create a animation of the monopod.
        for j = 1:n
            y = interp1(T' + linspace(0,1e-5,length(T)), Y, tFrame(j))';
            graphOUTPUT.update(y,tEvents,tFrame(j));
            f = getframe(gcf);
            im = frame2im(f); % Return image data associated with movie frame
            [imind,cm] = rgb2ind(im,256); % Convert RGB image to indexed image      
            % Write to the GIF File 
            if j == 1 
                imwrite(imind,cm,matfile,'gif','Loopcount',inf,'DelayTime',0.02); 
            else 
                imwrite(imind,cm,matfile,'gif','WriteMode','append','DelayTime',0.02); 
            end 
        %     if j == 1 
        %         imwrite(imind,cm,filename,'gif','Loopcount',inf,'DelayTime',0.04); 
        %     elseif j==n 
        %         imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',1); 
        %     else
        %         imwrite(imind,cm,filename,'gif','WriteMode','append','DelayTime',0.04); 
        %     end 
        end 

        
    end   
    
    
    
      if RecordVideo
        graphOUTPUT = SLIP_Model_Graphics_FM;
        graphFrame = [];
        tEvents = [tB_TD; tB_LO; tF_TD; tF_LO; tAPEX]; 
        n = round(T(end)*100); % # of frames per step
        tFrame = linspace(0, T(end), n+1);
        frameCount = 1;
        YFrame = [];
        NoE = zeros(n,1);
        v = VideoWriter('RunIPvT334','Motion JPEG AVI');
        open(v)
        % If desired, every iteration a rendered picture is saved to disc.  This
        % can later be used to create a animation of the monopod.
        for j = 1:n

            NoE(j) = sum((tEvents - tFrame(j))<0);

            y = interp1(T' + linspace(0,1e-5,length(T)), Y, tFrame(j))';
            
            if (j == 1) || (j == round(n/2))
             fig = gcf;
             YFrame = [YFrame, [ y; tFrame(j)] ];
             graphFrame{frameCount} = Plot_OneModel(fig,y,tEvents,tFrame(j));
             frameCount = frameCount + 1;
            elseif NoE(j)~=NoE(j-1)
             fig = gcf;
             YFrame = [YFrame, [ y; tFrame(j)] ];
             graphFrame{frameCount} = Plot_OneModel(fig,y,tEvents,tFrame(j));
             frameCount = frameCount + 1;
            end
            graphOUTPUT.update(y,tEvents,tFrame(j));
            % (un)comment the following line, if you (don't) want to save the individual frames to disc:
            % fig = gcf;
            % print(fig,'-r200','-djpeg',['MovieFrames/Frame',num2str(frameCount,'%04d.jpg')],'-opengl');
    %         print(fig,'-dpdf',['MovieFrames/Frame',num2str(frameCount,'%04d.pdf')]);
            writeVideo(v,getframe(gcf));
        end
  
        
        for k = 1:frameCount-1
            YFrame(1,k) = y(1) - 1.6*(frameCount - k); 
            graphFrame{k}.update(YFrame(1:end-1,k),tEvents,YFrame(end,k)); 
            writeVideo(v,getframe(gcf));
        end    
        
        
        close(v)
     end   
    
     if saveFig

        tB_TD = P(1);
        tB_LO = P(2);
        tF_TD = P(3);
        tF_LO = P(4);
        tAPEX = P(5);

        %% Compute GRFs
        if tB_TD < 0
             tB_TD = tB_TD + tAPEX;
        end
        if tB_TD > tAPEX
             tB_TD = tB_TD - tAPEX;
        end  
        if tB_LO < 0
             tB_LO = tB_LO + tAPEX;
        end
        if tB_LO > tAPEX
             tB_LO = tB_LO - tAPEX;
        end   
        if tF_LO < 0
             tF_LO = tF_LO + tAPEX;
        end
        if tF_LO > tAPEX
             tF_LO = tF_LO - tAPEX;
        end   
        if tF_TD < 0
             tF_TD = tF_TD + tAPEX;
        end
        if tF_TD > tAPEX
             tF_TD = tF_TD - tAPEX;
        end    

        tEvents = [tB_TD; tB_LO; tF_TD; tF_LO; tAPEX];  
        tUpdate = tEvents;
        tUpdate(end+1) =  tAPEX/2;
        tUpdate = sort(tUpdate);
        graphOUTPUT = SLIP_Model_Graphics_4Swing; 
        frameCount = 1;
        for k = 1:length(tUpdate)
            y = interp1(T' + linspace(0,1e-5,length(T)), Y, tUpdate(k))';
            graphOUTPUT.update(y,tEvents,tUpdate(k));
            % (un)comment the following line, if you (don't) want to save the individual frames to disc:
            fig = gcf;
    %         print(fig,'-r200','-djpeg',['MovieFrames/Frame',num2str(frameCount,'%04d.jpg')],'-opengl');
    %         print(fig,'-dpdf','-bestfit',['MovieFrames/Frame',num2str(frameCount,'%04d.pdf')]);

            print(fig,'-depsc',['MovieFrames/Frame',num2str(frameCount,'%04d.eps')]);
            frameCount = frameCount + 1;
        end 
     end    

end





% %     figure(3); clf; grid on;
%     graphOUTPUT = SLIP_Model_Graphics_4Swing;
%     fig = gcf;
%          Plot_OneModel(fig,y,tEvents,tFrame(j));
%          
%     tEvents = [tB_TD; tB_LO; tF_TD; tF_LO; tAPEX]; 
%     n = round(T(end)*100); % # of frames per step
%     tFrame = linspace(0, T(end), n+1);
%     frameCount = 1;
%     NoE = zeros(n,1);
%     % If desired, every iteration a rendered picture is saved to disc.  This
%     % can later be used to create a animation of the monopod.
%     for j = 1:n
%         
%         NoE(j) = sum((tEvents - tFrame(j))<0);
%         
%         y = interp1(T' + linspace(0,1e-5,length(T)), Y, tFrame(j))';
%         if j == 1 
%          fig = gcf;
%          Plot_OneModel(fig,y,tEvents,tFrame(j));
%         elseif NoE(j)~=NoE(j-1)
%          fig = gcf;
%          Plot_OneModel(fig,y,tEvents,tFrame(j));
%         end
%         graphOUTPUT.update(y,tEvents,tFrame(j));
%         % (un)comment the following line, if you (don't) want to save the individual frames to disc:
%         % fig = gcf;
%         % print(fig,'-r200','-djpeg',['MovieFrames/Frame',num2str(frameCount,'%04d.jpg')],'-opengl');
%         % print(fig,'-dpdf',['MovieFrames/Frame',num2str(frameCount,'%04d.pdf')]);
%         frameCount = frameCount + 1;
%     end
 
    
end

