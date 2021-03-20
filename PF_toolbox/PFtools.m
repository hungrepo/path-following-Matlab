% ================================================================================
% This is path following toolbox for testing path following algorithms in 2D cases
% ================================================================================
%    Details of the algorithms are described in the paper entittled:
%     
%       Theory, simulations, and experiments of path following guidance strategies for autonomous vehicles: Part I 
%    
%    Download at: http: ...
%
% Developed by: Nguyen T. Hung - IST Lisbon 
% Contact: https://nt-hung.github.io/
%
%
%==========================================================================

function PFtools
    clear all;
    close all;
%% Initialization
    T = 400;                                                                                        % Simulation time
    Ts = 0.2;                                                                                         % Sampling time
    N=T/Ts;
    t=0;
    % Initialize vehicle_position and orientation
        p0 = [15;-5]; psi0 = pi/2;
        x_robot0 = [p0;psi0];                                                                       % Robot state
    % Initialize the path 
        gamma0 = 0.1;                                                                                 % 
        pathtype = 'Bernoulli';                                                                       % path types includes : {Sin, circle, Polynominal,Bernoulli, Lawnmover, Heart}    
        [pd,d_pd,dd_pd,vd] = path_eq(pathtype);                                                     %                                                                  
    % Setup PF controller
        controller = 'Method 1';                                                                      % Controller {Method 1-Method 7}
    % Setup constraint for the vehicle input (speed and heading rate)
        umin = 0;     umax = 1;                         
        rmin = -0.2;  rmax = 0.2;
        vmin = -0.1;  vmax = 0.1;     
        l_bound = [umin;rmin;vmin];  u_bound = [umax;rmax;vmax];                             
    if strcmp(controller,'Method 5')||strcmp(controller,'Method 7')  
        MPC_PF = MPC_setup(Ts,d_pd,dd_pd,l_bound,u_bound,vd,controller);                              % Initilize an MPC object
    else
        MPC_PF=[];
    end
% Start main Loop =========================================================
    x_robot = x_robot0;
    x_path = [];
    upf = [0;0;0];                                                                                    % upf={u,r,v_gamma}
    gamma = gamma0;
for i = 1:N
    t = [t;i*Ts];
% Step 1: Get the state of the path
    p = x_robot(1:2,end);
    x_path(:,end+1) = path_state(pd,d_pd,dd_pd,controller,gamma(end),p);
% Step 2: Compute path following controller    
    upfi = PFcontrollers(x_robot(:,end),x_path(:,end),upf(:,end),controller,vd,Ts,MPC_PF,l_bound,u_bound);
    u_robot = upfi(1:2);  
% Step 3: Update the state of the vehicle    
    if (strcmp(controller,'Method 3'))||(strcmp(controller,'Method 4'))   
        [time y] = ode45(@(t,y) vehicle_model_2D_Type2(t,y,u_robot), [0, Ts], x_robot(1:2,end));
        x_robot(:,end+1) = [y(end,:)';u_robot(2)];
    else    
        [time y] = ode45(@(t,y) vehicle_model_2D_Type1(t,y,u_robot), [0, Ts], x_robot(:,end));
        x_robot(:,end+1) = y(end,:)';
    end    
    u_gamma = upfi(3);
    upf = [upf upfi];
% Step 4: Update the path parameter 
    if  (strcmp(controller,'Method 1'))||(strcmp(controller,'Method 3'))    
       gamma(end+1) = x_path(end,end);
    else
       gamma(end+1) = gamma(end)+Ts*u_gamma;
    end
end
%% Save Data to Workspace
    x_path = x_path';
    x_robot = x_robot';
    upf = upf';
    save_to_base(1);
%% Run animation
    pd = x_path(:,1:2);
    p = x_robot(:,1:2);
    animation_1vehicles
end
% End main loop ===========================================================

%% Vehicle kinematics_models
function dx = vehicle_model_2D_Type1(t,x,u)
%   In this model, input includes the vehicle speed and heading rate.
%   it is used for testing Method 1,2,5,6,7 
    psi = x(3);
    ur = u(1);
    r = u(2);
    dx = [ur*cos(psi);ur*sin(psi);r];
end
function dx = vehicle_model_2D_Type2(t,x,u)
%   In this model, input includes the vehicle speed and heading. 
%   it is used for testing Method 1,2,5,6,7
    psi = u(2);
    ur = u(1);
    dx = [ur*cos(psi);ur*sin(psi)];
end

