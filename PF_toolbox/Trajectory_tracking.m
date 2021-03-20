% =========================================================================
% Trajectory tracking for 2D cases
% =========================================================================
% This is trajectory tracking controller described in the thesis of Vanni-Vanni 

% Coded by: Nguyen T. Hung
% Contact: nt-hung.github.io


% Desired trajectory:  pd = [t; 10*sin(0.01*t); 
%
%==========================================================================

function TT_Controller
    clear all;
    close all;
%% Initialization
    T = 200;                                                                                        % Simulation time
    Ts = 0.1;                                                                                         % Sampling time
    N = T/Ts;
    time = [];
    % Initialize vehicle_position and orientation
        p0=[15;-5]; psi0=pi/2;
        x_vehicle0 = [p0;psi0];                                                                       % vehicle state                                                                                               % Controller {Method 1-Method 7}
    % Setup constraint for the vehicle input (speed and heading rate)
        umin=0;     umax=2;                                                         % limit on the vehicle's speed
        rmin=-0.5;  rmax=0.5;                                                       % limit on the vehicle's heading rate
        l_bound=[umin;rmin];  u_bound=[umax;rmax];                             
% Start main Loop =========================================================
    x_vehicle = x_vehicle0;
    x_traj = [];
    x_traj_dot = [];
    u_vehicle = [];                                                                      % upf={u,r}
for i = 0:N
    t = i*Ts;
    time(end+1) = i*Ts;
% Step 1: update the desired trajectory
    x_traj(:,end+1) = [t; 10*sin(0.05*t)];                                          % desired trajectory
    x_traj_dot(:,end+1) = [1; 0.5*cos(0.05*t)];                                     % derivative of trajectory
% Step 2: Compute trajectory tracking controller    
    u_vehicle(:,end+1) = TT_controllers(x_vehicle(:,end),x_traj(:,end),x_traj_dot(:,end),l_bound,u_bound);  
% Step 3: Update the state of the vehicle    
    [time y]=ode45(@(t,y) vehicle_model_2D(t,y,u_vehicle(:,end)), [0, Ts], x_vehicle(:,end));
    x_vehicle(:,end+1) = y(end,:)';
end
%% Save Data to Workspace
    x_traj = x_traj';
    x_vehicle = x_vehicle';
    u_vehicle = u_vehicle';
 %   save_to_base(1);
%% Plot
plot(x_vehicle(:,1),x_vehicle(:,2));
hold on;
plot(x_traj(:,1),x_traj(:,2));
legend('vehicle trajectory', 'desired trajectory'); 
end
% End main loop ===========================================================

%% Vehicle_model_2D_Type1 dynamics
function dx = vehicle_model_2D(t,x,u)
%   in this model, input includes the vehicle speed and heading rate 
    psi=x(3);
    ur=u(1);
    r=u(2);
    dx=[ur*cos(psi);ur*sin(psi);r];
end
function u_TT= TT_controllers(x_vehicle,x_traj,x_traj_dot,l_bound,u_bound)
 delta=-0.5;
 Delta= [1    0;
         0  -delta];
 Delta_inv=inv(Delta);        
 epsilon=[delta; 0];        
 kx=.2; ky=0.05;    
 Kk=[kx 0;
     0  ky];
p = x_vehicle(1:2);
psi = x_vehicle(3);
pd  = x_traj;
d_pd = x_traj_dot; 

RB_I=[cos(psi)   -sin(psi);
      sin(psi)    cos(psi)]; 
e_pos=RB_I'*(p-pd)-epsilon;
% TT control law
u_TT = Delta_inv*(-Kk*e_pos+RB_I'*d_pd); 
u_TT = max(min(u_TT,u_bound),l_bound);
end