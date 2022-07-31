% ================================================================================
% This is path following toolbox for testing path following algorithms in 2D cases
% ================================================================================
%    Details of the algorithms are described in the paper entittled:
%     
%       A review of path following control strategies for autonomous robotic vehicles: theory, simulations, and experiments
%    
%    Authors: Nguyen Hung, Francisco Rego, Joao Quintas, Joao Cruz, Marcelo Jacinto, David Souto, André Potes, Luis Sebastião and António Pascoal 

% The code was written by: Nguyen Hung  
% See more the work: https://nt-hung.github.io/
%
%
%=================================================================================

function PFtools
    clear all;
    close all;
%% Initialization
    T = 500;                                                                                        % Simulation time
    Ts = 0.2;                                                                                         % Sampling time
    N=T/Ts;
    t=0;
    % Initialize vehicle_position and orientation
        p0 = [25;-15]; psi0 = pi/2;
        x_robot0 = [p0;psi0];                                                                   % Robot state
    % Set the path 
        gamma0 = 0.1;                                                                                % 
        path_type = 'Bernoulli';                                                                       % path types includes : {Sin, circle, polynominal,Bernoulli, Lawnmover, Heart}    
        [pd,d_pd,dd_pd,vd] = path_eq(path_type);                                                     %                                                                  
    % Set PF controller
        controller = 'Method 1';                                                                % Controller {Method 1-Method 7}
    % Setup constraint for the vehicle input (speed and heading rate)
        umin = 0;     umax = 1;                         
        rmin = -0.2;  rmax = 0.2;
        vmin = -1;  vmax = 1;     
        l_bound = [umin;rmin;vmin];  u_bound = [umax;rmax;vmax];                             
    if strcmp(controller,'Method 5')||strcmp(controller,'Method 7')  
        MPC_PF = MPC_setup(Ts,d_pd,dd_pd,l_bound,u_bound,vd,controller);                              % Initilize an MPC object
    else
        MPC_PF=[];
    end
%% Start main Loop =========================================================
    x_robot = x_robot0;
    x_path = [];
    upf = [0;0;0];                                                                                    % upf={u,r,v_gamma}
    gamma = gamma0;
    compute_time = [];
    path_compute_time = [];
for i = 1:N
    t = [t;i*Ts];
% Step 1: Get the state of the path
    p = x_robot(1:2,end);
    tic 
    x_path(:,end+1) = path_state(pd,d_pd,dd_pd,controller,gamma(end),p);
    path_compute_time(i+1) = toc;
% Step 2: Compute path following controller   
    [upfi exc_time] = PFcontrollers(x_robot(:,end),x_path(:,end),upf(:,end),controller,vd,Ts,MPC_PF,l_bound,u_bound);
    compute_time(i+1) = exc_time + path_compute_time(end) ;   
    u_robot = upfi(1:2);  
% Step 3: Update the state of the vehicle    
    if (strcmp(controller,'Method 3'))||(strcmp(controller,'Method 4'))   
        model_type = 'Type II';
    else
        model_type = 'Type I';
    end    
    x_robot(:,end+1) = vehicle_models(x_robot(:,end),u_robot,model_type,Ts);
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
%    plot_results;
end
% End main loop ===========================================================


