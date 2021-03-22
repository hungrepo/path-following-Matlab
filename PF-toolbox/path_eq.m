%% Define path equation function
% ==========================================================================
% Inputs: pathtype
% Outputs: 
%       pd:     path equation
%       d_pd:   partial derivative of pd respect to gamma
%       dd_pd:  partial derivative of d_pd respecto gamma
% =========================================================================
function [pd,d_pd,dd_pd,vd]=path_eq(pathtype)
syms t;                                             % We use t instead of for the simplicity in presentation
        if strcmp(pathtype,'line');
            vd = 0.02;                                % Desired speed profile for dot_gamma
            a = 30; x0 = 0; y0 = 0;
            pd=[x0+a*t+1e-10*cos(t);
                y0+0];
        end
        if strcmp(pathtype,'circle');
            vd = 0.05;
            a = 10; x0 = 0; y0 = 0;
            pd=[x0+a*cos(t);
                y0+a*sin(t)];
        end
        if strcmp(pathtype,'sin');
            vd = 0.2; 
            a = 10;omega = 0.05; phi = 0; d = 10;                                                                            
            pd = [a*sin(omega*t+phi)+d;
                  t];
        end
        if strcmp(pathtype,'polynominal');
            vd = 0.005;
            a=[0;0;-1.797;9.905;10.891;1]; b=[0;0;-23.539;37.078;-3.539;1]; c=0; 
            x_offset = 20;
            y_offset = -10;
            z=t+c;
            phi=[z^5;z^4;z^3;z^2;z;1];
            pd=[a'*phi + x_offset;
                b'*phi + y_offset];
        end
        if strcmp(pathtype,'Bernoulli');
            vd = 0.02;
            a = 20;
            z = 1+sin(t)^2;
            pd = [a*cos(t)/z;
                  a*sin(t)*cos(t)/z];
        end
        if strcmp(pathtype,'Heart');
            vd = 0.02;
            pd = [16*sin(t)^3;
                  13*cos(t)-5*cos(2*t)-2*cos(3*t)-cos(4*t)];
        end     
   d_pd  = diff(pd) ;                          % partial derivative of pd respect to t 
   dd_pd = diff(d_pd);                         % partial  
   pd = matlabFunction(pd);                    % convert to matlab handle function  
   d_pd = matlabFunction(d_pd);
   dd_pd = matlabFunction(dd_pd);
end        