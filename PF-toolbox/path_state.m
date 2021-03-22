%% Path function
%   Input:      type of path, gamma, t 
%   Output:     x_path= [pd,pd_gamma,psi_P,kappa]
%   pd:         The position of the reference point
%   pd_gamma:   partial derivative of pd_gamma
%   psiP:      Angle of the tangent of the path
%   kammpa:     curvature of the path
% =========================================================================
function x_path=path_state(pd,d_pd,dd_pd,controller,gamma,p)
    persistent psiP_old  psiP_out_old 
    if (strcmp(controller,'Method 1')) || (strcmp(controller,'Method 3'))
        % find the point on the path closet to the vehicle
        f=@(t)(pd(t)-p)'*d_pd(t);
        gamma_opt = fzero(f,gamma);
    else
        gamma_opt=gamma;                                        % just use the last gamma
    end
  
    %% Compute the first and second derivative of pd with the optimal gamma
    t=gamma_opt(1);
    pd=pd(t);
    pd_gamma=d_pd(t);
    b=dd_pd(t);                                               
    kappa=(pd_gamma(1)*b(2)-pd_gamma(2)*b(1))/norm(pd_gamma)^3;                      % curvature of the path 
    
    %% Open the space of the angle to R    
        psiP_new = atan2(pd_gamma(2), pd_gamma(1));
        if isempty(psiP_old)
            psiP_old = psiP_new;
            psiP_out_old = psiP_new;
            psiP_out = psiP_new;
        else
            psiP_out = alg_convert(psiP_new,psiP_old,psiP_out_old);
        end
        psiP_old = psiP_new;
        psiP_out_old = psiP_out;
        
    %%     
    x_path=[pd;pd_gamma;kappa;psiP_out;gamma_opt];
    
end