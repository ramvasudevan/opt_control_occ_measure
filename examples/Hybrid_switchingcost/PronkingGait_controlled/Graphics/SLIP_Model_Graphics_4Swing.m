% *************************************************************************
% classdef SLIP_Model_Graphics(p) < OutputCLASS
%
% Two dimensional graphics of a SLIP model.
%
% The graphics object must be initialized with the vector of system
% parameters p.
%
%
% Properties: - NONE
% Methods:    - NONE
%
%
% Created by C. David Remy on 07/10/2011
% MATLAB 2010a - Windows - 64 bit
%
% Documentation:
%  'A MATLAB Framework For Gait Creation', 2011, C. David Remy (1), Keith
%  Buffinton (2), and Roland Siegwart (1),  International Conference on
%  Intelligent Robots and Systems, September 25-30, San Francisco, USA 
%
% (1) Autonomous Systems Lab, Institute of Robotics and Intelligent Systems, 
%     Swiss Federal Institute of Technology (ETHZ) 
%     Tannenstr. 3 / CLA-E-32.1
%     8092 Zurich, Switzerland  
%     cremy@ethz.ch; rsiegwart@ethz.ch
%
% (2) Department of Mechanical Engineering, 
%     Bucknell University
%     701 Moore Avenue
%     Lewisburg, PA-17837, USA
%     buffintk@bucknell.edu
%
%   See also OUTPUTCLASS.
%
classdef SLIP_Model_Graphics_4Swing < OutputCLASS 
    % Private attributes:
    properties 
        fig; % The output window 
        % Patch objects used in the graphical representation
        Body;  
        Leg_B;
        Leg_F;
    end
    % Public methods:
    methods
        % Constructor:
        function obj = SLIP_Model_Graphics_4Swing
            obj.slowDown = 1;      % Run this in real time.
            obj.rate     = 0.05;   % with 25 fps 
            % Copy the parameter vector:

            
            % Initialize the graphics
            obj.fig = figure(1);
%             clf(obj.fig);
            % Set window properties
            set(obj.fig,'Name','SLIP model');  % Window title
            set(obj.fig,'Color','w');          % Background color
            set(obj.fig, 'Renderer','OpenGL');
%             set(obj.fig, 'position', get(0,'ScreenSize'));

%             % Set up view:
%             box on;
%             axis equal;
%             axis([-1,2,-0.5,2.5]);
            set(obj.fig, 'position', [100 100 750 600]);
            axis off;
%             obj.fig.PaperPositionMode = 'auto';
%             fig_pos =  obj.fig.PaperPosition;
%             obj.fig.PaperSize = [fig_pos(3) fig_pos(4)];
            ax = gca;
            outerpos = ax.OuterPosition;
            ti = ax.TightInset; 
            left = outerpos(1) + ti(1);
            bottom = outerpos(2) + ti(2);
            ax_width = outerpos(3) - ti(1) - ti(3);
            ax_height = outerpos(4) - ti(2) - ti(4);
%             ax.Position = [left bottom ax_width ax_height];
            box off;
            axis equal;
               
            % Draw the ground. It reaches from -2.5 to +6.5.
            h   = 0.01; % Height of the bar at the top
            n   = 5000;  % Number of diagonal stripes in the shaded area
            s   = 0.05; % Spacing of the stripes
            w   = 0.01; % Width of the stripes
            ext = 0.1;  % Length of the stripes
    
            % Create vertices by shifting a predefined pattern 'n' times to the right:
            v = [     -50,0;
                repmat([     0,    -h],n,1) + [-50+linspace(0,s*n,n)',zeros(n,1)];
                repmat([  -ext,-ext-h],n,1) + [-50+linspace(0,s*n,n)',zeros(n,1)];
                repmat([-ext+w,-ext-h],n,1) + [-50+linspace(0,s*n,n)',zeros(n,1)];
                repmat([     w,    -h],n,1) + [-50+linspace(0,s*n,n)',zeros(n,1)];
                -50+s*n+w,0];
            % Connect to faces:
            f = [1,2,4*n+1,4*n+2;
                 repmat([0,n,2*n,3*n],n,1) + repmat((1:n)',1,4)+1];

            
            % Define some arbitrary states:
            x        = 1;
            y        = 1.2;
            l_leg    = 1;
            phi_body = 0;
                        
            
            % Draw Ground
            vert_x_out = [-15 100 100 -15];
            vert_y_out = [0 0 -20 -20];
            patch(vert_x_out, vert_y_out,'white');   
            patch('faces', f, 'vertices', v); 
            
            % The representation of the main body:
            obj.Body  = DrawBody([x,y,phi_body]);
            % The representation of the back right leg:
            obj.Leg_B = DrawLegs([x-0.5;y],l_leg, 0.3);
            % The representation of the front right leg:
            obj.Leg_F = DrawLegs([x+0.5;y],l_leg,-0.1);
            

        end   
        % Updated function.Is called by the integrator:
        function obj = update(obj,y,tEvents,T)
            
            
            [LegLength, LegAngle, BodyJPos, BackJPos, FrontJPos] = ComputeJoint_LegLA(y,tEvents,T);


            % The representation of the body as ellipse object:
            SetBody(BodyJPos, obj.Body);
            
            % back right Leg
            SetLegs(BackJPos,LegLength.B,LegAngle.B,obj.Leg_B);
            % front right Leg
            SetLegs(FrontJPos,LegLength.F,LegAngle.F,obj.Leg_F);
            
            
%             axis([-1.5 + BodyJPos(1),1.5 + BodyJPos(1),-0.1,2]);
            axis([-1.5, 7, -0.1, 2]);
%             axis([-2 ,10,-0.2,2]);
            drawnow();
        end
    end
end