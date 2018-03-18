% Simulator
% ====== States ======
% Y(1)      -- x
% Y(2)      -- dx
% Y(3)      -- y
% Y(4)      -- dy
% Y(5)      -- phi
% Y(6)      -- dphi
% Y(7)      -- alpha_L (rear leg)
% Y(8)      -- dalpha_L
% Y(9)      -- alpha_R (front leg)
% Y(10)     -- dalpha_R
% ====== Modes ======
% 00 -- Flight phase
% 01 -- Right leg touch-down
% 10 -- Left leg touch-down
% 11 -- Double support
% 
clear;

Polyflag = 1;

T_start = 0;
Y_start = [0;1.01391108000981;1.59710245806802;-0.001;0;-0;0.00001;0.897058474278427;0;0.897058474278342]';

Y_start = [0;1.01391108000981;1.69710245806802;-1.14752891246160e-17;9.47580313540214e-14;1.57031038452452e-18;-2.36679877941720e-13;0.897058474278427;-4.77206184996964e-13;0.897058474278342]'; % <- this is from Hop
% Y_start = [0;1.01391108000981;1.59710245806802;-1.14752891246160e-17;9.47580313540214e-14;1.57031038452452e-18;-2.36679877941720e-13;0.8;-4.77206184996964e-13;0.8]'; % <- this is from Hop

% Y_start = [0;1.05887503063298;1.59568928338079;-1.09348784495598e-17;9.06119513003635e-14;1.39494980518933e-18;-2.29824877057264e-13;0.934378146148562;-4.77962875059735e-13;0.934378146148479]'; % <- this is from Gallop
% Y_start = [ 0; 1.0139; 1.5971; -1e15; 0; 0; 0; 0.57146; 0; 0.5709 ]';
% Y_start = [0; 1; 1.6; -0.033368; 0; 0.0001; 0.19275; 0.12969; 0.18726; 0.12172]';
% Y_start = [x0, dx0, y0, dy0, phi0, dphi0, alphaB0, dalphaB0, alphaF0, dalphaF0];
T = [];
Y = [];
switching_seq = [];
switching_time = [];
mode_seq = [];
T_max = 10;

% Y_start = [Y_start, 0];

current_mode = 0;
stop_flag = false;

if Polyflag
%     D_FF = @(tt,xx) [ xx(2); FF_Dyn_poly(xx(2:end)) ];
    D_FF = @(tt,xx) [ xx(2); FF_Dyn_poly(xx(2:end)); -xx(3) ];
    D_FR = @(tt,xx) [ xx(2); FR_Dyn_poly(xx(2:end)) ];
    D_LF = @(tt,xx) [ xx(2); LF_Dyn_poly(xx(2:end)) ]; 
    D_LR = @(tt,xx) [ xx(2); LR_Dyn_poly(xx(2:end)) ];
    
    Rst_L = @(xx) [ xx(1); Reset_L_poly(xx(2:end)) ];
    Rst_R = @(xx) [ xx(1); Reset_R_poly(xx(2:end)) ];
    
    ode_options_FF = odeset( 'Events', @FF_PronkingGaitEventsFcn_poly );
    ode_options_FR = odeset( 'Events', @FR_PronkingGaitEventsFcn_poly );
    ode_options_LF = odeset( 'Events', @LF_PronkingGaitEventsFcn_poly );
    ode_options_LR = odeset( 'Events', @LR_PronkingGaitEventsFcn_poly );
else
    D_FF = @(tt,xx) [ xx(2); FF_Dyn(xx(2:end)) ];
    D_FR = @(tt,xx) [ xx(2); FR_Dyn(xx(2:end)) ];
    D_LF = @(tt,xx) [ xx(2); LF_Dyn(xx(2:end)) ];
    D_LR = @(tt,xx) [ xx(2); LR_Dyn(xx(2:end)) ];
    
    Rst_L = @(xx) [ xx(1); Reset_L(xx(2:end)) ];
    Rst_R = @(xx) [ xx(1); Reset_R(xx(2:end)) ];
    
    ode_options_FF = odeset( 'Events', @FF_PronkingGaitEventsFcn );
    ode_options_FR = odeset( 'Events', @FR_PronkingGaitEventsFcn );
    ode_options_LF = odeset( 'Events', @LF_PronkingGaitEventsFcn );
    ode_options_LR = odeset( 'Events', @LR_PronkingGaitEventsFcn );
end


while ( T_start < T_max ) && ( ~stop_flag )
    mode_seq = [ mode_seq; current_mode ];
    
    switch current_mode
        case 0  % FF
            Y_start = [Y_start, 0 ];
            [T_part,Y_part,te,ye,ie] = ode45(D_FF,[T_start,T_max],Y_start,ode_options_FF);
%             if ie(end) == 5
%                 stop_flag = true;
%             end
%             if length(ie) ~= 1
% %                 disp( ie );
%                 current_mode = 3;
%                 Y_start = Rst_R( Rst_L( Y_part(end,:)' ) )';
%             else
            disp(Y_part(end,end));
            Y_part = Y_part( :, 1:end-1 );
            y = ye( 1, 1:end-1 );
            switch ie( 1 )
                case 1  % Left leg TD
                    current_mode = 2;
                    Y_start = Rst_L( y' )';
                case 3  % Right leg TD
                    current_mode = 1;
                    Y_start = Rst_R( y' )';
                case 5  % Apex
                    stop_flag = true;
            end
%             T_part = T_part( 1 : find(te(1) == T_part) );
%             Y_part = Y_part( 1 : find(te(1) == T_part), : );
        case 1  % FR
            [T_part,Y_part,te,ye,ie] = ode45(D_FR,[T_start,T_max],Y_start,ode_options_FR);
            y = ye( 1, : );
            switch ie( 1 )
                case 1  % Left leg TD
                    current_mode = 3;
                    Y_start = Rst_L( y' )';
                case 4  % Right leg LO
                    current_mode = 0;
                    Y_start = y;
            end
            T_part = T_part( 1 : find(te(1) == T_part) );
            Y_part = Y_part( 1 : find(te(1) == T_part), : );
        case 2  % LF
            [T_part,Y_part,te,ye,ie] = ode45(D_LF,[T_start,T_max],Y_start,ode_options_LF);
            y = ye( 1, : );
            switch ie( 1 )
                case 2  % Left leg LO
                    current_mode = 0;
                    Y_start = y;
                case 3  % Right leg TD
                    current_mode = 3;
                    Y_start = Rst_R( y' )';
            end
            T_part = T_part( 1 : find(te(1) == T_part) );
            Y_part = Y_part( 1 : find(te(1) == T_part), : );
        case 3  % LR
            [T_part,Y_part,te,ye,ie] = ode45(D_LR,[T_start,T_max],Y_start,ode_options_LR);
            y = ye( 1, : );
            switch ie( 1 )
                case 2  % Left leg LO
                    current_mode = 1;
                    Y_start = y;
                case 4  % Right leg LO
                    current_mode = 2;
                    Y_start = y;
            end
    end
    
    T = [ T; T_part ];
    Y = [ Y; Y_part ];
    T_start = T_part(end);
    switching_seq = [ switching_seq; ie( 1 ) ];
    switching_time = [ switching_time; T_start ];
    
end

P = zeros( 1, 5 );
P( switching_seq ) = switching_time;

ShowTrajectory( T,Y,[P, zeros(1,3)] );
