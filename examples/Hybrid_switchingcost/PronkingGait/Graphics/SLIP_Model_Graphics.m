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
classdef SLIP_Model_Graphics < OutputCLASS 
    % Private attributes:
    properties 
        % The output window
        fig; 
        % The parameter vector:
        p;
        % Patch and line objects used in the graphical representation
%         COGPatch;
        Body;
        
        SpringLine_bl;
        SpringLine_br;
        SpringLine_fl;
        SpringLine_fr;
    end
    % Public methods:
    methods
        % Constructor:
        function obj = SLIP_Model_Graphics(p)
            obj.slowDown = 1; % Run this in real time.
            obj.rate     = p(11);   % with 25 fps
            
            % Copy the parameter vector:
            obj.p = p;
            
            % Initialize the graphics
            obj.fig = figure();
            clf(obj.fig);
            % Set some window properties
            set(obj.fig,'Name','2D-Output of a SLIP model');  % Window title
            set(obj.fig,'Color','w');         % Background color
            set(obj.fig, 'position', get(0,'ScreenSize'));



            % Define some arbitrary states:
            x = 1;
            y = 1.2;
            l_leg = 1;
            [~, ~, ~] = SystParamDefinition();
            phi_body =0;
                        

           

            % The representation of the back left leg as a line object:
            obj.SpringLine_bl = DrawLegs(x,y,l_leg,phi_body);

            % The representation of the front left leg as a line object
            obj.SpringLine_fl =  DrawLegs(x,y,l_leg,phi_body);
            
            
             obj.Body = DrawBody(x,y,phi_body);
            % The representation of the back right leg as a line object:
            obj.SpringLine_br = DrawLegs(x,y,l_leg,phi_body);

            % The representation of the front right leg as a line object:
            obj.SpringLine_fr = DrawLegs(x,y,l_leg,phi_body);
            
                
           % Draw the ground. It reaches from -0.5 to +8.5.
            h   = 0.01; % Height of the bar at the top
            n   = 180;  % Number of diagonal stripes in the shaded area
            s   = 0.05;  % Spacing of the stripes
            w   = 0.01; % Width of the stripes
            ext = 0.1;  % Length of the stripes
    
            % Create vertices by shifting a predefined pattern 'n' times to the right:
%             v = [-0.5,0;
%                  repmat([0,-h],n,1) + [-0.5+linspace(0,s*n,n)',zeros(n,1)];
%                  repmat([-ext,-ext-h],n,1) + [-0.5+linspace(0,s*n,n)',zeros(n,1)];
%                  repmat([-ext+w,-ext-h],n,1) + [-0.5+linspace(0,s*n,n)',zeros(n,1)];
%                  repmat([w,0-h],n,1) + [-0.5+linspace(0,s*n,n)',zeros(n,1)];
%                  -0.5+s*n+w,0];
                 v = [-1.5,0;
                 repmat([0,-h],n,1) + [-1.5+linspace(0,s*n,n)',zeros(n,1)];
                 repmat([-ext,-ext-h],n,1) + [-1.5+linspace(0,s*n,n)',zeros(n,1)];
                 repmat([-ext+w,-ext-h],n,1) + [-1.5+linspace(0,s*n,n)',zeros(n,1)];
                 repmat([w,0-h],n,1) + [-1.5+linspace(0,s*n,n)',zeros(n,1)];
                 -1.5+s*n+w,0];
            % Connect to faces:
            f = [1,2,4*n+1,4*n+2;
                 repmat([0,n,2*n,3*n],n,1) + repmat((1:n)',1,4)+1];
            % Color is uniformly black
            patch('faces', f, 'vertices', v);
            
            % Set up view:
            box on
            grid on
            axis equal
            axis([-1,8.5,-0.5,2.5])
        end
        
        
        % Updated function.  Is called by the integrator:
        function obj = update(obj, y, z, ~, ~)
           
            % Get a mapping for the state and parameter vectors.  This allows us
            % to use a more readable syntax: "y(contStateIndices.dy)" instead of
            % "y(3)" while still operating with vectors and not with structs.
            % We keep the index-structs in memory to speed up processing
            persistent contStateIndices  systParamIndices discStateIndices
            if isempty(contStateIndices) || isempty(discStateIndices) || isempty(systParamIndices) 
                [~, ~, contStateIndices] = ContStateDefinition();
                [~, ~, discStateIndices] = DiscStateDefinition();
                [~, ~, systParamIndices] = SystParamDefinition();
            end
            
            Hip_x_b = -obj.p(systParamIndices.COM)*2*obj.p(systParamIndices.a)*cos(y(contStateIndices.phi)) + y(contStateIndices.x);
            Hip_x_f = (1-obj.p(systParamIndices.COM))*2*obj.p(systParamIndices.a)*cos(y(contStateIndices.phi)) + y(contStateIndices.x);
            Hip_y_b = -obj.p(systParamIndices.COM)*2*obj.p(systParamIndices.a)*sin(y(contStateIndices.phi)) + y(contStateIndices.y);
            Hip_y_f = (1-obj.p(systParamIndices.COM))*2*obj.p(systParamIndices.a)*sin(y(contStateIndices.phi)) + y(contStateIndices.y);
            a = obj.p(systParamIndices.a)*1.2;
            
            % Evaluate states:
            % Leg configuration:

            
            
    % Phase for Back Left feet  *******************************************        
    switch z(discStateIndices.blPhase) 
        case 1 % wait over, ready for contact
            l_leg_bl    = obj.p(systParamIndices.l_0);
            gamma_leg_bl = obj.p(systParamIndices.angAttb); 
        case 2 % at stance
            l_leg_bl = sqrt((Hip_x_b-z(discStateIndices.blcontPt))^2 + (Hip_y_b)^2);
            gamma_leg_bl = atan2(z(discStateIndices.blcontPt)-Hip_x_b, Hip_y_b);
        case 3 % left ground, wait for touchdown
            l_leg_bl    = obj.p(systParamIndices.l_0);
            gamma_leg_bl = -obj.p(systParamIndices.angAttb);   
    end
    
   % Phase for Back Right feet  *******************************************                          
   switch z(discStateIndices.brPhase) 
        case 1 % wait over, ready for contact
            l_leg_br    = obj.p(systParamIndices.l_0);
            gamma_leg_br = obj.p(systParamIndices.angAttb); 
        case 2 % at stance
            l_leg_br = sqrt((Hip_x_b-z(discStateIndices.brcontPt))^2 + (Hip_y_b)^2);
            gamma_leg_br = atan2(z(discStateIndices.brcontPt)-Hip_x_b, Hip_y_b);
        case 3 % left ground, wait for touchdown
            l_leg_br    = obj.p(systParamIndices.l_0);
            gamma_leg_br = -obj.p(systParamIndices.angAttb);    
    end                            
                            
   % Phase for Front Left feet  *******************************************                          
   switch z(discStateIndices.flPhase) 
        case 1 % wait over, ready for contact
            l_leg_fl    = obj.p(systParamIndices.l_0);
            gamma_leg_fl = obj.p(systParamIndices.angAttf);
        case 2 % at stance
            l_leg_fl = sqrt((Hip_x_f-z(discStateIndices.flcontPt))^2 + (Hip_y_f)^2);
            gamma_leg_fl = atan2(z(discStateIndices.flcontPt)-Hip_x_f, Hip_y_f);
        case 3 % left ground, wait for touchdown
            l_leg_fl   = obj.p(systParamIndices.l_0);
            gamma_leg_fl = -obj.p(systParamIndices.angAttf);  
    end                               
                            
    % Phase for Front Right feet  *****************************************                         
    switch z(discStateIndices.frPhase) 
        case 1 % wait over, ready for contact
            l_leg_fr   = obj.p(systParamIndices.l_0);
            gamma_leg_fr = obj.p(systParamIndices.angAttf);
        case 2 % at stance
            l_leg_fr = sqrt((Hip_x_f-z(discStateIndices.frcontPt))^2 + (Hip_y_f)^2);
            gamma_leg_fr = atan2(z(discStateIndices.frcontPt)-Hip_x_f, Hip_y_f);
        case 3 % left ground, wait for touchdown
            l_leg_fr  =obj.p(systParamIndices.l_0);
            gamma_leg_fr = -obj.p(systParamIndices.angAttf);
    end                                 
           

            % Careful, the following overwrites the state vector y with the
            % horizontal position y:
            Delta_x_b = -obj.p(systParamIndices.COM)*2*obj.p(systParamIndices.a)*cos(y(contStateIndices.phi)) + obj.p(systParamIndices.COM)*2*obj.p(systParamIndices.a);
            Delta_x_f = (1-obj.p(systParamIndices.COM))*2*obj.p(systParamIndices.a)*cos(y(contStateIndices.phi)) - (1-obj.p(systParamIndices.COM))*2*obj.p(systParamIndices.a);
            Delta_y_b = -obj.p(systParamIndices.COM)*2*obj.p(systParamIndices.a)*sin(y(contStateIndices.phi));
            Delta_y_f = (1-obj.p(systParamIndices.COM))*2*obj.p(systParamIndices.a)*sin(y(contStateIndices.phi)); 
            phi_body = y(contStateIndices.phi);
            x = y(contStateIndices.x);
            y = y(contStateIndices.y);
            
             % COG:
%             phi = linspace(0, pi/2, 10);
%             vert_x = [0,sin(phi)*0.1,0];
%             vert_x = [vert_x;vert_x;-vert_x;-vert_x]' + x;
%             vert_y = [0,cos(phi)*0.1,0];
%             vert_y = [vert_y;-vert_y;-vert_y;vert_y]' + y;
%             set(obj.COGPatch,'xData', vert_x, 'yData',vert_y);
            

            % back left Leg
            xbl=x - obj.p(systParamIndices.a)*2*obj.p(systParamIndices.COM) + Delta_x_b;
            ybl=y + Delta_y_b;
            SetDrawLegs(xbl,ybl,l_leg_bl,gamma_leg_bl,obj.SpringLine_bl)
            % front left Leg
            xfl=x + obj.p(systParamIndices.a)*2*(1-obj.p(systParamIndices.COM)) - Delta_x_f;
            yfl=y + Delta_y_f;
            SetDrawLegs(xfl,yfl,l_leg_fl,gamma_leg_fl,obj.SpringLine_fl)

            
            % The representation of the body as ellipse object:
            SetDrawBody(x,y, phi_body ,obj.Body)
            % back right Leg
            xbr=x - obj.p(systParamIndices.a)*2*obj.p(systParamIndices.COM) + Delta_x_b;
            ybr=y + Delta_y_b;
            SetDrawLegs(xbr,ybr,l_leg_br,gamma_leg_br,obj.SpringLine_br)

            % front right Leg
            xfr=x + obj.p(systParamIndices.a)*2*(1-obj.p(systParamIndices.COM)) - Delta_x_f;
            yfr= y + Delta_y_f;
            SetDrawLegs(xfr,yfr,l_leg_fr,gamma_leg_fr,obj.SpringLine_fr)
            

            
            axis([-1,3,-0.2,1.5])
            drawnow();
        end
    end
end