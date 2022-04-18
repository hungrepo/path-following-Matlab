function upf=PFcontrollers(x_robot,x_path,upf,controller,vd,Ts,MPC_PF,l_bound,u_bound)

p=x_robot(1:2);
psi=x_robot(3);

pd=x_path(1:2);
pd_gamma=x_path(3:4);
hg=sqrt(pd_gamma(1)^2+pd_gamma(2)^2);
v_robot=upf(1);r_robot=upf(2);v_gamma=upf(3);
psiP=x_path(6);  
kappa=x_path(5);
gamma=x_path(end);
%% Compute PF error 
if  strcmp(controller,'Method 1')     
    upf=Method1(p,psi,pd,psiP,kappa,v_robot,hg,vd);
    upf(2)=sat(upf(2),l_bound(2),u_bound(2));
elseif strcmp(controller,'Method 2')     
    upf=Method2(p,psi,pd,psiP,kappa,v_gamma,v_robot,hg,vd);
    upf(2:3)=sat(upf(2:3),l_bound(2:3),u_bound(2:3));
elseif strcmp(controller,'Method 3')   
    upf=Method3(p,psi,pd,psiP,kappa,v_robot,hg,vd);
elseif strcmp(controller,'Method 4')   
    upf=Method4(p,psi,pd,psiP,kappa,v_robot,hg,vd);
    upf(3)=sat(upf(3),l_bound(3),u_bound(3));
elseif strcmp(controller,'Method 5')   
    upf=Method5(p,psi,pd,psiP,kappa,v_robot,hg,vd,gamma,MPC_PF);    
elseif strcmp(controller,'Method 6')
    upf=Method6(p,psi,pd,pd_gamma,psiP,kappa,v_robot,hg,vd,Ts);
        upf=sat(upf,l_bound,u_bound);
elseif strcmp(controller,'Method 7')
    upf= Method7(p,psi,pd,pd_gamma,psiP,vd,Ts,gamma,MPC_PF);        
end


end
function upf= Method1(p,psi,pd,psiP,kappa,v_robot,hg,vd)

%     T=x_path(9:10);
%     N=x_path(11:12);
    RI_F=[cos(psiP)      sin(psiP);                                         % From {I} to {F}
           -sin(psiP)      cos(psiP)]; 
%    RI_F=[T N]';                                         % From {I} to {F]; 

    e_P=RI_F*(p-pd);
%    e_P1=RI_F1*(p-pd);
    s1=e_P(1);
    y1=e_P(2);
    psie=psi-psiP; 
    
    [k1,k2,k3,k_delta,theta,Delta_h]=ConPara1();

    delta=-theta*tanh(k_delta*y1);
    ydot=v_robot*sin(psie);
    delta_dot=-theta*k_delta*(1-(tanh(k_delta*y1))^2)*ydot;
    psi_tilde=psie-delta;
    % Controller
    %u_d=.5;
    u_d=hg*vd;
    u=u_d;
    uP=u*cos(psie)/(1-kappa*y1);
    v_gamma=uP/hg;
    if psie==delta
       r=delta_dot-k5*y1*v_robot+kappa*v_gamma*hg;    
    else
       r=kappa*uP + delta_dot - k1*psi_tilde - k2*y1*u*(sin(psie) - sin(delta))/psi_tilde;
    end
    upf=[u;r;v_gamma];
end   
function upf= Method2(p,psi,pd,psiP,kappa,v_gamma,v_robot,hg,vd)

%     T=x_path(9:10);
%     N=x_path(11:12);
     RI_F=[cos(psiP)      sin(psiP);                                         % From {I} to {F}
           -sin(psiP)      cos(psiP)]; 
%    RI_F=[T N]';                                         % From {I} to {F]; 

    e_P=RI_F*(p-pd);
%    e_P1=RI_F1*(p-pd);
    s1=e_P(1);
    y1=e_P(2);
    psie=psi-psiP; 
    
    [k1,k2,k3,k_delta,theta,Delta_h]=ConPara1();

    delta=-theta*tanh(k_delta*y1);
    ydot=v_robot*sin(psie)-hg*kappa*v_gamma*s1;
    delta_dot=-theta*k_delta*(1-(tanh(k_delta*y1))^2)*ydot;
    psi_tilde=psie-delta;
    % Controller
    %u_d=.5;
    u_d=hg*vd;
    u=u_d;
    uP=(u*cos(psie)+k3*s1);
    v_gamma=uP/hg;
    if psie==delta
       r=delta_dot-k5*y1*v_robot+kappa*v_gamma*hg;    
    else
       r=kappa*uP + delta_dot - k1*psi_tilde - k2*y1*u*(sin(psie) - sin(delta))/psi_tilde;
    end
    upf=[u;r;v_gamma];
end    
function upf= Method3(p,psi,pd,psiP,kappa,v_robot,hg,vd)

%     T=x_path(9:10);
%     N=x_path(11:12);
     RI_F=[cos(psiP)      sin(psiP);                                         % From {I} to {F}
           -sin(psiP)      cos(psiP)]; 
%    RI_F=[T N]';                                         % From {I} to {F]; 

    e_P=RI_F*(p-pd);
%    e_P1=RI_F1*(p-pd);
    s1=e_P(1);
    y1=e_P(2);
     
    [k1,k2,k3,k_delta,theta,Delta_h]=ConPara1();
    % Controller
    %u_d=.5;
    u_d=hg*vd;
    u=u_d;
    v_gamma=0;
    psi_los=psiP+atan(-y1/Delta_h); 
    upf=[u;psi_los;v_gamma];
end   
function upf= Method4(p,psi,pd,psiP,kappa,v_robot,hg,vd)

%     T=x_path(9:10);
%     N=x_path(11:12);
     RI_F=[cos(psiP)      sin(psiP);                                         % From {I} to {F}
           -sin(psiP)      cos(psiP)]; 
%    RI_F=[T N]';                                         % From {I} to {F]; 

    e_P=RI_F*(p-pd);
%    e_P1=RI_F1*(p-pd);
    s1=e_P(1);
    y1=e_P(2);
    psie=psi-psiP; 
    [k1,k2,k3,k_delta,theta,Delta_h]=ConPara1();
    % Controller
    u_d=hg*vd;
    u=u_d;
    uP=u*cos(psie)+k3*s1;
    v_gamma=uP/hg;
    psi_los=psiP+atan(-y1/Delta_h); 
    upf=[u;psi_los;v_gamma];
end   
function upf= Method5(p,psi,pd,psiP,kappa,v_robot,hg,vd,gamma,MPC_PF)
    persistent w0 lbw ubw lbg ubg; 
    if isempty(w0)
        w0=MPC_PF.w0;      
        lbw=MPC_PF.lbw;        ubw=MPC_PF.ubw;
        lbg=MPC_PF.lbg;        ubg=MPC_PF.ubg;
    end

%     T=x_path(9:10);
%     N=x_path(11:12);
     RI_F=[cos(psiP)      sin(psiP);                                         % From {I} to {F}
           -sin(psiP)      cos(psiP)]; 
%    RI_F=[T N]';                                         % From {I} to {F]; 

    e_P=RI_F*(p-pd);
%    e_P1=RI_F1*(p-pd);
    s1=e_P(1);
    y1=e_P(2);
    psie=psi-psiP;
    
    u_d=hg*vd;
    u=u_d;
    
    x0=[s1;y1;psie;gamma];
    nx=length(x0);
    w0(1:nx)=x0;
    lbw(1:nx)=x0;
    ubw(1:nx)=x0;
    sol = MPC_PF.nlp('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                'lbg', lbg, 'ubg', ubg);
    w_opt = full(sol.x);
    u_mpc=w_opt(5:6);
    r_d=u_mpc(1);
    v_gamma=u_mpc(2);
    w0=w_opt;
    upf=[u;r_d;v_gamma];
end   
function upf= Method6(p,psi,pd,pd_gamma,psiP,kappa,v_robot,hg,vd,Ts)
 %%% Tunning parameters (This can be done as initialization)
    persistent gamma_dot_old;
    if isempty(gamma_dot_old)
    gamma_dot_old=0;
    end
    [delta,Delta_inv,epsilon,Kk,kz]=ConPara2();
    %%%-----------------------------
    RB_I=[cos(psi)   -sin(psi);
          sin(psi)    cos(psi)]; 
    e_pos=RB_I'*(p-pd)-epsilon;
    e_gamma=gamma_dot_old-vd;

    %%% PF control law
    ud=Delta_inv*(-tanh(Kk*e_pos)+RB_I'*pd_gamma*vd); 
    gamma_ddot=-kz*e_gamma+e_pos'*RB_I'*pd_gamma;
    %%% 
    gamma_ddot=sat(gamma_ddot, -0.005, 0.005);
    gamma_dot=gamma_dot_old+Ts*gamma_ddot;
    gamma_dot=sat(gamma_dot, -0.005, 0.2);
    gamma_dot_old=gamma_dot;
    upf=[ud;gamma_dot];
end
function upf= Method7(p,psi,pd,pd_gamma,psiP,vd,Ts,gamma,MPC_PF)       
 %%% Tunning parameters (This can be done as initialization)
persistent w0 lbw ubw lbg ubg; 
    if isempty(w0)
        w0=MPC_PF.w0;      
        lbw=MPC_PF.lbw;        ubw=MPC_PF.ubw;
        lbg=MPC_PF.lbg;        ubg=MPC_PF.ubg;
    end

   [delta,Delta_inv,epsilon,Kk,kz]=ConPara2();
   
    RB_I=[cos(psi)   -sin(psi);
          sin(psi)    cos(psi)]; 
    eB=RB_I'*(p-pd)-epsilon;
    
    x0=[eB;psi;gamma];
    nx=length(x0);
    w0(1:nx)=x0;
    lbw(1:nx)=x0;
    ubw(1:nx)=x0;
    sol = MPC_PF.nlp('x0', w0, 'lbx', lbw, 'ubx', ubw,...
                'lbg', lbg, 'ubg', ubg);
    w_opt = full(sol.x);
    upf=w_opt(5:7);
    w0=w_opt;
end
function [k1,k2,k3,k_delta,theta,Delta_h]=ConPara1()
% Control parameters for Method 1 to Method 5
    k1=1;
    k2=1;
    k3=0.1;
    theta=0.1*pi/4;
    k_delta=2;
    Delta_h=5;
end
function [delta,Delta_inv,epsilon,Kk,kz]=ConPara2()
% Control parameters for Method 6 and Method 7
    delta=-0.2;
    Delta= [1    0;
            0  -delta];
    Delta_inv=inv(Delta);        
    epsilon=[delta; 0];        
    kx=.1; ky=0.05;    
    Kk=[kx 0;
        0  ky];
    kz=1;
end
function y=sat(u,l_bound,u_bound)
    y=max(min(u,u_bound),l_bound);
end
 