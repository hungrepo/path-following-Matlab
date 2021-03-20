function test
syms t;
xd=@(t) t;
yd=@(t)10*sin(t);
p=[pi/2;-15];
pd=@(t)[xd(t);yd(t)];
%pd
dist=@(t)norm(pd(t)-p);
x0=1;
A=[-1;
    1]; 
b=[10 20*pi];
S  = fmincon(dist,x0,A,b);

xd_opt=xd(S);
yd_opt=yd(S);
x=0:0.1:2*pi;
xd_out=xd(x);
yd_out=yd(x);
hold on;
plot(xd_out,yd_out);
plot(xd_opt,yd_opt,'*');
plot(p(1),p(2),'o')
end