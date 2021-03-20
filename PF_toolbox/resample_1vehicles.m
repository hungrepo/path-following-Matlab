
% m=600;
m=5;
N=round((size(p,1)-1)/m);
t1=zeros(N,2);
t2=zeros(N,2);
t3=zeros(N,2);
t4=zeros(N,2);
t5=zeros(N,2);
t6=zeros(N,2);
t7=zeros(N,2);
t8=zeros(N,2);
t9=zeros(N,2);
k=1;
for i=0:N-1;
t7(i+1,:)=pd(m*i+k,:);
t8(i+1,:)=p(m*i+k,:); 
t9(i+1,:)=yaw(m*i+k,:); 

end
pd=t7;
p=t8;
yaw=t9;

