%% Vehicle kinematics_models
function x_robot_next = vehicle_models(x_robot,u_robot,model_type, Ts)
    if strcmp(model_type,'Type I')   
        [time y] = ode45(@(t,y) vehicle_model_2D_Type1(t,y,u_robot), [0, Ts], x_robot);
         x_robot_next = y(end,:)';
    elseif strcmp(model_type,'Type II')
        [time y] = ode45(@(t,y) vehicle_model_2D_Type2(t,y,u_robot), [0, Ts], x_robot(1:2));
         x_robot_next = [y(end,:)';u_robot(2)];
    else
        display('Model type is not compatible');
        return;
        x_robot_next = zeros(length(x_robot));
    end
end
function dx = vehicle_model_2D_Type1(t,x,u)
%   In this model, input includes the vehicle speed and heading rate.
%   it is used for testing Methods 1,2,5,6,7 
    psi = x(3);
    ur = u(1);
    r = u(2);
    dx = [ur*cos(psi);ur*sin(psi);r];
end
function dx = vehicle_model_2D_Type2(t,x,u)
%   In this model, input includes the vehicle speed and heading. 
%   it is used for testing Methods 3,4
    psi = u(2);
    ur = u(1);
    dx = [ur*cos(psi);ur*sin(psi)];
end
