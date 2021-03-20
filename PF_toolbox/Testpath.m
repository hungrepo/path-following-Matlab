clear all;
gamma=0:1:61;
% a=[0.0000; 0; 0; 1; 1; 0];
% b=[0.000; 0; 0; 0; 1; 0];

%a=[0;0;0;0;1;0]; 
c=238;
b=[0;0; -3.255259096425168e-04; 0.263729636233269;-72.550756644896480; 6.505073497062662e+03]; 
x=[]; y=[]; psid=[];
for i=1:length(gamma)
 z=gamma(i)+c;
 x=[x; z];
 y=[y; b(3)*z^3+b(4)*z^2+b(5)*z+b(6)];
 %psid=[psid;atan2(dphi'*b(1:5),dphi'*a(1:5))];
end
plot(x,y);
%figure
%plot(gamma,psid);

% Gamma = 0:5:61;
% c = 238;
% for i = 1:length(Gamma)
% x = Gamma(i) + c;
% y = ((-0.000325*(Gamma (i)+c)^3)+(-0.2637* (Gamma (i)+c)^2)+ (72.558 * (Gamma (i)+c))-6505);
% end
% plot (x,y)

