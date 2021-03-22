%% Path function
%   Input:      type of path, gamma, t 
%   Output:     x_path= [pd,pd_gamma,psi_P,kappa]
%   pd:         The position of the reference point
%   pd_gamma:   partial derivative of pd_gamma
%   psiP:      Angle of the tangent of the path
%   kammpa:     curvature of the path

function x_path=paths(pathtype,gamma,t)
    persistent psiP_old  psiP_out_old 
        if strcmp(pathtype,'Circle');
           [pd,pd_gamma,kappa]=circle(gamma,t); 
        end
        if strcmp(pathtype,'Sin');
           [pd,pd_gamma,kappa]=sinpath(gamma,t); 
        end
        if strcmp(pathtype,'Polynominal');
           [pd,pd_gamma,kappa]=polynominal(gamma,t); 
        end
        if strcmp(pathtype,'Bernoulli');
           [pd,pd_gamma,kappa]=Bernoulli(gamma,t); 
        end
        if strcmp(pathtype,'Lawnmover');
           [pd,pd_gamma,kappa]=lawnmover(gamma,t); 
        end
        if strcmp(pathtype,'Heart');
           [pd,pd_gamma,kappa]=heart(gamma,t); 
        end
    %% Open the space of the angle to R    
        psiP_new = atan2(pd_gamma(2), pd_gamma(1));
        if t == 0
            psiP_old = psiP_new;
            psiP_out_old = psiP_new;
            psiP_out = psiP_new;
        else
            psiP_out = alg_convert(psiP_new,psiP_old,psiP_out_old);
        end
        psiP_old = psiP_new;
        psiP_out_old = psiP_out;
    
    x_path=[pd;pd_gamma;kappa;psiP_out];
end

%% ************************************************************************
function [pd,pd_gamma,kappa]=sinpath(gamma,t)
%% Path equation
%       x=a*sin(omega*gamma+phi)+d;
%       y=gamma;
    a=10;omega=0.05;phi=0;d=10;                                                                            
    pd=[a*sin(omega*gamma+phi)+d;
        gamma];
    pd_gamma=[a*omega*cos(omega*gamma+phi);
              1];
    kappa=a*omega^2*sin(omega*gamma+phi)/(norm(pd_gamma)^3);
 end
%% ************************************************************************
function [pd,pd_gamma,kappa]=circle(gamma,t)
%% Path equation
%       x=x0+a*cos(gamma);
%       y=y0+a*sin(gamma);
        a=10; x0=0; y0=0;
%% Clockwise        
    pd=[x0+a*cos(gamma);
        y0+a*sin(gamma)];
    pd_gamma=[-a*sin(gamma);
               a*cos(gamma)];
    kappa=1/a;
    
%    T=[-sin(gamma);cos(gamma)];                 % tangent vector;
%    N=[-cos(gamma);-sin(gamma)];                % normal vector;
%    N=[-T(2);T(1)];
%% counter clockwise    
%     pd=[x0+a*cos(gamma);y0-a*sin(gamma)];
%     pd_gamma=[-a*sin(gamma);-a*cos(gamma)];
%     hg=a;
%     kappa=1/a;
%     T=[-sin(gamma); -cos(gamma)];                 % tangent vector;
%  %   N=[-cos(gamma);  sin(gamma)];                % normal vector;
%     N=[-T(2);T(1)];
%     if t==0
%         psiP0=atan2(pd_gamma(2),pd_gamma(1));
%     end
%     if t==0
%         x_path=[pd;psiP0;pd_gamma;hg;kappa;gamma;T;N];
%     else
%         x_path=[pd;psiP;pd_gamma;hg;kappa;gamma;T;N];
%     end
end
%% ************************************************************************
function [pd,pd_gamma,kappa]=polynominal(gamma,t)
%% Path equation
%       a=[a1;a2;a3;a4;a5;a6]; b=[b1;b2;b3;b4;b5;b6];
%       phi=[(gamma+c)^5;(gamma+c)^4;(gamma+c)^3;(gamma+c)^2;(gamma+c);1]
%       x=a'*phi;
%       y=b'*phi;
    % Path from yogang
%         a=[0;0;0;0;1;0];
%         b=[0;0;-3.255e-04; 0.263;-72.550; 6.505e+03]; % parameters of polynominal path
%         c=0;
    % Path from Bahareh
%        a=pathpar(:,1);b=pathpar(:,2);
       %  a=[-25.0647738972264;70.4704087877383;-76.2420352989336;32.7607709906985;7.04154149110126; 1];
       % b=[-25.8120636399255;60.5162465481219;-26.4811860051878;-16.2546057813239;7.05800506702106; 1]; % parameters of polynominal path
    a=[0;0;-1.797;9.905;10.891;1]; b=[0;0;-23.539;37.078;-3.539;1]; c=0;  
    z=gamma+c;
    phi=[z^5;z^4;z^3;z^2;z;1];
    dphi=[5*z^4;4*z^3;3*z^2;2*z;1];
    ddphi=[20*z^3;12*z^2;6*z;2];
    pd=[a'*phi;
        b'*phi];
    xdot_gamma=a(1:5)'*dphi;     ydot_gamma=b(1:5)'*dphi;
    xddot_gamma=a(1:4)'*ddphi;   yddot_gamma=b(1:4)'*ddphi;
    pd_gamma=[xdot_gamma;
              ydot_gamma];
    kappa=(xdot_gamma*yddot_gamma-ydot_gamma*xddot_gamma)/(norm(pd_gamma)^3);

end
%% ************************************************************************
function [pd,pd_gamma,kappa]=Bernoulli(gamma,t)
%% Path equation
%   x=a*cos(gamma)/(1+(sin(gamma))^2);
%   y=a*sin(gamma)*cos(gamma)/(1+sin(gamma)^2);
    a=20;
    z=1+sin(gamma)^2;
    xd=a*cos(gamma)/z;
    yd=a*sin(gamma)*cos(gamma)/z;
    xdot_gamma =(a*sin(gamma)*(sin(gamma)^2 - 3))/z^2;
    ydot_gamma=-(a*(3*sin(gamma)^2 - 1))/(sin(gamma)^2 + 1)^2;
    pd=[xd;
        yd];
    pd_gamma=[xdot_gamma;
              ydot_gamma];
    num=3*sqrt(2)*cos(gamma);
    den=a*sqrt(3-cos(2*gamma));
    kappa=num/den;
end
%% ************************************************************************
function [pd,pd_gamma,kappa]=lawnmover(gamma,t)
% 
    a=30; R1=10; R2=10; d=0;

    if gamma<=1
        xd=a*gamma; yd=d;
        pd=[xd;yd];
        pd_gamma=[a;0];
        kappa=0;
    elseif (1<gamma)&&(gamma<=2)
        z=gamma-1;
        xd=a+(R1-d)*sin(z*pi); yd=d+(R1-d)*(1-cos(z*pi));
        pd=[xd;yd];
        pd_gamma=[(R1-d)*pi*cos(z*pi);(R1-d)*pi*sin(z*pi)];
        kappa=1/(R1-d); 
    elseif (2<gamma)&&(gamma<=3)                         % (2<gamma)&&(gamma<=3)
        z=gamma-2;
        xd=a*(1-z); yd=2*R1-d;
        pd=[xd;yd];
        pd_gamma=[-a;0];
        kappa=0;
    elseif (3<gamma)&&(gamma<=4)
        z=gamma-3;
        xd=0-(R2+d)*sin(z*pi); yd=2*R1-d+(R2+d)*(1-cos(z*pi));
        pd=[xd;yd];
        pd_gamma=[-(R2+d)*pi*cos(z*pi);(R2+d)*pi*sin(z*pi)];
        kappa=-1/R2;    
    else
        z=gamma-4;
        xd=a*z; yd=2*(R1+R2)+d;
        pd=[xd;yd];
        pd_gamma=[a;0];
        kappa=0;
    end

     
end
%% ************************************************************************
 function [pd,pd_gamma,kappa]=heart(gamma,t)
% x=a*sin(gamma);
% y=b*cos(gamma)-c*cos(2gamma)-d*cos(3gamma)-cos(4gamma);
%  a=20;
    z=gamma;                                                                % 
    xd=16*sin(z)^3;
    yd=13*cos(z)-5*cos(2*z)-2*cos(3*z)-cos(4*z);
    xdot_gamma =24*sin(2*z)*sin(z);
    xddot_gamma=24*(2*cos(2*z)*sin(z)+sin(2*z)*cos(z));
    ydot_gamma =-13*sin(z)+10*sin(2*z)+6*sin(3*z)+4*sin(4*z);
    yddot_gamma=-13*cos(z)+20*cos(2*z)+18*cos(3*z)+16*cos(4*z);
    pd=[xd;yd];
    pd_gamma=[xdot_gamma;ydot_gamma];
    kappa=(xdot_gamma*yddot_gamma-ydot_gamma*xddot_gamma)/(norm(pd_gamma)^3);
end
