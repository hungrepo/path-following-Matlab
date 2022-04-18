function MPC_PFC=MPC_setup(Ts,d_pd,dd_pd,l_bound,u_bound,vd,controller)
    import casadi.*
    N = 10;                      % number of control intervals
    gamma=SX.sym('gamma');
    d_pd  = feval(d_pd,gamma);
    dd_pd = feval(dd_pd,gamma);
    hg=norm(d_pd);
    kappa=(d_pd(1)*dd_pd(2)-d_pd(2)*dd_pd(1))/hg^3; 

    if (strcmp(controller,'Method 5'))
        MPC_PFC=NLP_Method5(Ts,d_pd,dd_pd,hg,kappa,gamma,l_bound,u_bound,vd,N);
    end
    if (strcmp(controller,'Method 7'))
        MPC_PFC=NLP_Method7(Ts,d_pd,dd_pd,hg,kappa,gamma,l_bound,u_bound,vd,N);
    end
end

function  MPC_PFC = NLP_Method5(Ts,d_pd,dd_pd,hg,kappa,gamma,l_bound,u_bound,vd,N)
    import casadi.*
    T = Ts*N;                       % Time horizon
    % system state
    s1 = SX.sym('s1');
    y1 = SX.sym('y1');
    psie=SX.sym('psie');
    x = [s1;y1;psie;gamma];
    nx=length(x);
    % Input
    r = SX.sym('r');
    v_g=SX.sym('v_g');
    u=[r;v_g];
    nu=length(u);
%% path following system

xdot = [vd*hg*cos(psie)-v_g*hg*(1-kappa*y1);...
        vd*hg*sin(psie)-kappa*hg*s1*v_g; ...
        r-kappa*hg*v_g;...
        v_g];
% subject to constraint
    rmax=u_bound(2);rmin=l_bound(2);
    vmax=u_bound(3);vmin=l_bound(3);
    umax=[rmax;vmax];
    umin=[rmin;vmin];
    
%% Objective term
Q=diag([1 5 .1]);
R=diag([1 1]);
ua=[vd*cos(psie)-v_g ;
    r-kappa*hg*v_g];
L = [s1 y1 psie]*Q*[s1;y1;psie]+ ua'*R*ua;

%% Continuous time dynamics
f = Function('f', {x, u}, {xdot, L});

%% Formulate discrete time dynamics


   % Fixed step Runge-Kutta 4 integrator
   M = 4; % RK4 steps per interval
   DT = T/N/M;
   f = Function('f', {x, u}, {xdot, L});
   X0 = MX.sym('X0', nx);
   U = MX.sym('U',nu);
   X = X0;
   Q = 0;
   for j=1:M
       [m1, m1_q] = f(X, U);
       [m2, m2_q] = f(X + DT/2 * m1, U);
       [m3, m3_q] = f(X + DT/2 * m2, U);
       [m4, m4_q] = f(X + DT * m3, U);
       X=X+DT/6*(m1 +2*m2 +2*m3 +m4);
       Q = Q + DT/6*(m1_q + 2*m2_q + 2*m3_q + m4_q);
    end
    F = Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});

%% Evaluate the model at a test point

%% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

%% Formulate the NLP
Xk = X0;
xmax=[inf;inf;inf;inf];
xmin=[-inf;-inf;-inf;-inf];
uzero=[0;0];
xzero=[0;0;0;0];

% "Lift" initial conditions
X0 = MX.sym('X0', 4);
w = {w{:}, X0};
lbw = [lbw; xzero];
ubw = [ubw; xzero];
w0 = [w0; xzero];

% Formulate the NLP

Xk = X0;
for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)],nu);
    w = {w{:}, Uk};
    lbw = [lbw; umin];
    ubw = [ubw;  umax];
    w0 = [w0;  uzero]; 

    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_end = Fk.xf;
    J=J+Fk.qf;

    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], nx);
    w = {w{:}, Xk};
    lbw = [lbw; xmin];
    ubw = [ubw; xmax];
    w0 = [w0; xzero];

    % Add equality constraint
    g = {g{:}, Xk_end-Xk};
    lbg = [lbg; xzero];
    ubg = [ubg; xzero];
    
    % Add inequality constraint
end
%% contractive inequality constraint 
%   
%% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
options = struct('ipopt',struct('print_level',0),'print_time',false);
MPC_PFC.nlp = nlpsol('solver', 'ipopt', prob, options);
MPC_PFC.w0=w0;
MPC_PFC.lbw=lbw;
MPC_PFC.ubw=ubw;
MPC_PFC.lbg=lbg;
MPC_PFC.ubg=ubg;
% w0(1:4)=x0;
% % Initialize guess
% u0=[0;0];
% for i=1:N
%     Fk = F('x0',x0,'p',u0);
%     x0=full(Fk.xf);
%     w0(6*i+1:6*i+4)=x0;
%     w0(6*i-1:6*i)=u0;
% end
end
function  MPC_PFC = NLP_Method7(Ts,d_pd,dd_pd,hg,kappa,gamma,l_bound,u_bound,vd,N)
import casadi.*
T = Ts*N; % Time horizon
% state of the path following system
eB = SX.sym('eB',2);
psi=SX.sym('psi');
x = [eB;psi;gamma];
nx=length(x);
% Input
u_v=  SX.sym('u_v');
r = SX.sym('r');
v_g=SX.sym('v_g');
u=[u_v;r;v_g];
nu=length(u);

%% Model equations - Path following error
% path folllowing equaiton
S=[0 -r;
   r 0];

delta=-0.2;
Delta= [1    0;
        0   -delta];
RIB=[ cos(psi) sin(psi)
     -sin(psi) cos(psi)];
 
xdot = [S*eB+Delta*[u_v;r]-RIB*d_pd*v_g;...
        r; ...
        v_g];
% subject to constraint
umax=u_bound;
umin=l_bound;
%% Objective term
Q = diag([.5 .2]);
R = diag([10 10]);
ub = Delta*[u_v;r]-RIB*d_pd*v_g;
L = eB'*Q*eB+ub'*R*ub+1*(v_g-vd)^2;
%% Continuous time dynamics
f = Function('f', {x, u}, {xdot, L});

%% Formulate discrete time dynamics


   % Fixed step Runge-Kutta 4 integrator
   M = 4; % RK4 steps per interval
   DT = T/N/M;
   f = Function('f', {x, u}, {xdot, L});
   X0 = MX.sym('X0', nx);
   U = MX.sym('U',nu);
   X = X0;
   Q = 0;
   for j=1:M
       [m1, m1_q] = f(X, U);
       [m2, m2_q] = f(X + DT/2 * m1, U);
       [m3, m3_q] = f(X + DT/2 * m2, U);
       [m4, m4_q] = f(X + DT * m3, U);
       X=X+DT/6*(m1 +2*m2 +2*m3 +m4);
       Q = Q + DT/6*(m1_q + 2*m2_q + 2*m3_q + m4_q);
    end
    F = Function('F', {X0, U}, {X, Q}, {'x0','p'}, {'xf', 'qf'});

%% Evaluate the model at a test point

%% Start with an empty NLP
w={};
w0 = [];
lbw = [];
ubw = [];
J = 0;
g={};
lbg = [];
ubg = [];

%% Formulate the NLP
Xk = X0;
xmax=[inf;inf;inf;inf];
xmin=[-inf;-inf;-inf;-inf];
uzero=[0;0;0];
xzero=[0;0;0;0];

% "Lift" initial conditions
X0 = MX.sym('X0', 4);
w = {w{:}, X0};
lbw = [lbw; xzero];
ubw = [ubw; xzero];
w0 = [w0; xzero];

% Formulate the NLP
Xk = X0;
for k=0:N-1
    % New NLP variable for the control
    Uk = MX.sym(['U_' num2str(k)],nu);
    w = {w{:}, Uk};
    lbw = [lbw; umin];
    ubw = [ubw; umax];
    w0 = [w0;  uzero]; 


    % Integrate till the end of the interval
    Fk = F('x0', Xk, 'p', Uk);
    Xk_end = Fk.xf;
    J=J+Fk.qf;

    % New NLP variable for state at end of interval
    Xk = MX.sym(['X_' num2str(k+1)], nx);
    w = {w{:}, Xk};
    lbw = [lbw; xmin];
    ubw = [ubw; xmax];
    w0 = [w0; xzero];

    % Add equality constraint
    g = {g{:}, Xk_end-Xk};
    lbg = [lbg; xzero];
    ubg = [ubg; xzero];
    
    % Add inequality constraint
end
%% contractive inequality constraint 
%   
%% Create an NLP solver
prob = struct('f', J, 'x', vertcat(w{:}), 'g', vertcat(g{:}));
options = struct('ipopt',struct('print_level',0),'print_time',false);
MPC_PFC.nlp = nlpsol('solver', 'ipopt', prob, options);
MPC_PFC.w0=w0;
MPC_PFC.lbw=lbw;
MPC_PFC.ubw=ubw;
MPC_PFC.lbg=lbg;
MPC_PFC.ubg=ubg;
% w0(1:4)=x0;
% % Initialize guess
% u0=[0;0];
% for i=1:N
%     Fk = F('x0',x0,'p',u0);
%     x0=full(Fk.xf);
%     w0(6*i+1:6*i+4)=x0;
%     w0(6*i-1:6*i)=u0;
% end
end
