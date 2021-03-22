
function x_path=path_state(path_type,gamma)
%% Path function
%   Input:       pathtype: circle, line, sine, polynominnal
%                gamma
%   Output:      x_path= [pd,pd_gamma,psi_P,kappa]
%                   pd:     the position of the reference point on the path
%                   d_pd:   partial derivative of pd respect to gamma
%                   psiP:   Angle of the tangent of the path at gamma
%                   kammpa: curvature of the path at gamma
%                   vd:     desired speed profile along the path        
% =========================================================================
    persistent psiP_old  psiP_out_old                                       % memorized/static variable
    if path_type=='circle'
%       path equation, for each type of path we have each 
%       equation here, the below is an example for for circle path (arc), 
%       I will give you another equation for other type paths later
        a=10;                       % radius of the circle
        pd=a*[cos(omega*gamma);
              sin(omega*gamma)];
    end
%   compute the first partial derivative of pd respect to gamma
    d_pd=a*[-omega*sin(omega*gamma);
             omega*cos(omega*gamma)];
%   compute the second partial derivative of pd respect to gamma        
    dd_pd=a*[-omega*sin(omega*gamma);
              omega*cos(omega*gamma)];    
%   compute the curvature of the path at gamma                                              
    kappa=(d_pd(1)*dd_pd(2)-d_pd(2)*dd_pd(1))/norm(d_pd)^3;                      
%   compute desired speed profile along the path for gamma
    vd=0.01;            % this is just an example, it can be a constant or a function of gamma
%   compute the angle of the tangent of the path makes with x_I (North)
    psiP_new = atan2(pd_gamma(2), pd_gamma(1));
%   Open the space of this angle to real - this is VERY important for method 1 to Method 6    
%   We need to apply this algoirtm for the vehicle heading as well
        if isempty(psiP_old)     
            % in the first interation 
            psiP_old = psiP_new;
            psiP_out_old = psiP_new;
            psiP = psiP_new;
        else
            psiP = alg_convert(psiP_new,psiP_old,psiP_out_old);
        end
        psiP_old = psiP_new;
        psiP_out_old = psiP;
        
%%  Return   
    x_path=[pd;d_pd;kappa;psiP;gamma;vd];

%   x_path will be the input of path following controllers. 
%   In console it is enough to plot pd - the reference point on the path to 
%   to compare with the position of the vehicle,
%   to see if the path following method work well or not
end


function alg_out= alg_convert(alg_new, alg_old, alg_out_old)
%   This algorithm is used to open the space of yaw angle and path agle
%   to real - this is VERY important for method 1 to Method 6    
% ========================================================================= 
alg_e = alg_new - alg_old;
    if (alg_e > 3 * pi / 2)
        alg_out = alg_out_old - 2 * pi + alg_e;
    elseif (alg_e < -3 * pi / 2)
        alg_out = alg_out_old + 2 * pi + alg_e;
    else
        alg_out = alg_out_old + alg_e;
    end
end
