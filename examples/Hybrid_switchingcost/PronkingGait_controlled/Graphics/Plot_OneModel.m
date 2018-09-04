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
classdef Plot_OneModel < OutputCLASS 
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
        function obj = Plot_OneModel(FigHandle,y,tEvents,T)

            
            % Initialize the graphics
            obj.fig = FigHandle;
       
            
            [LegLength, LegAngle, BodyJPos, BackJPos, FrontJPos] = ComputeJoint_LegLA(y,tEvents,T);
            % The representation of the front right leg:
            obj.Leg_F = DrawLegs(FrontJPos,LegLength.F, LegAngle.F);
            obj.Leg_F.L_Sp1.EdgeAlpha  = 0.3;
            obj.Leg_F.L_UpB.EdgeAlpha  = 0.3;
            obj.Leg_F.L_Upo.EdgeAlpha  = 0.3;
            obj.Leg_F.L_Ups.EdgeAlpha  = 0.3;
            obj.Leg_F.L_low.EdgeAlpha  = 0.3;
            obj.Leg_F.L_Sp2.EdgeAlpha  = 0.3;
            
            fields = fieldnames(obj.Leg_F);    
            for i=1:numel(fields)
                uistack(obj.Leg_F.(fields{numel(fields) + 1 - i}),'bottom');
            end    

            % The representation of the back right leg:
            obj.Leg_B = DrawLegs(BackJPos,LegLength.B, LegAngle.B);
            obj.Leg_B.L_Sp1.EdgeAlpha  = 0.3;
            obj.Leg_B.L_UpB.EdgeAlpha  = 0.3;
            obj.Leg_B.L_Upo.EdgeAlpha  = 0.3;
            obj.Leg_B.L_Ups.EdgeAlpha  = 0.3;
            obj.Leg_B.L_low.EdgeAlpha  = 0.3;
            obj.Leg_B.L_Sp2.EdgeAlpha  = 0.3;

            fields = fieldnames(obj.Leg_B);    
            for i=1:numel(fields)
                uistack(obj.Leg_B.(fields{numel(fields) + 1 - i}),'bottom');
            end
            

            % The representation of the main body:
            obj.Body  = DrawBody(BodyJPos);
            obj.Body.B_bg.EdgeAlpha  = 0.8;
            obj.Body.B_bg.FaceAlpha  = 0.3;
            uistack(obj.Body.B_bg,'bottom');
            obj.Body.B_sha.EdgeAlpha  = 0.3;
            obj.Body.B_sha.FaceAlpha  = 0.3;
            uistack(obj.Body.B_sha,'bottom');
            obj.Body.B_out.EdgeAlpha  = 0.3;
            obj.Body.B_out.FaceAlpha  = 0.3;
            uistack(obj.Body.B_out,'bottom');

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
            
            
        end
    end
end