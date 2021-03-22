clear all;
close all;

pd=x_path(:,1:2);
p=x_robot(:,1:2);
yaw=x_robot(:,3)*180/pi;

n_end=length(yaw);

    
    
%% Patch vehicle to the mission    
    resample_1vehicles;
    animation_3vehicles_LMH;    % animation_3vehicles_LMH; for circle
    
%     str = ' $$ v_{cx}=0m/s$$ \\ $$v_{cy}=0m/s$$' ;
%     text(-45,30, str,'Interpreter','latex')
%     
    dim = [0.2 0.5 0.3 0.3];
    str = {'$$v_{cx}=0m/s$$','$$v_{cy}=0m/s$$'};
    annotation('textbox',dim,'String',str,'Interpreter','latex','FitBoxToText','on');
    
    %% Plot evolution of rabit
    fig3= figure(3);
    subplot(2,1,1);
    plot(t,ug1,'b');hold on; plot(t,ug2,'r');
    tmin=t(1)-1; tmax=t(end)+1;
    limit=[tmin tmax -1 1];
    axis(limit);
    title('Speed the vitual reference');
%     ylabel('sdot[m/s]');
    ylabel('$$\dot{s}[m/s]$$','Interpreter','latex');
    xlabel('$$t[second]$$','Interpreter','latex');
    grid on;
    
    subplot(2,1,2);
    plot(t,rd1,'b');hold on; plot(t,rd2,'r');
    tmin=t(1)-1; tmax=t(end)+1;
    limit=[tmin tmax -.4 .4];
    axis(limit);
    title('Heading rate');
    ylabel('$$r_{d}[degree/s]$$','Interpreter','latex');
    xlabel('$$t[second]$$','Interpreter','latex');
    grid on;
     
    
load sin_x02_c
n=1;
nend=2000;    % for circle
%  nend=1700;   % for eight
%  nend=2000;   % for lawn
p1=p1(n:nend,:);
pd1=pd1(n:nend,:);
yaw1=yaw1(n:nend);
rd1=rd1(n:nend)*pi/180;
rmax=0.3;
rd_max=rmax*ones(1,nend);
rd_min=-rmax*ones(1,nend);
vcxy_hat1=vcxy_hat1(n:nend,:);
ug1=ug1(n:nend);

ugmax=0.8;
ug_max=ugmax*ones(1,nend);
ug_min=-ugmax*ones(1,nend);



p2=p2(n:nend,:);
pd2=pd2(n:nend,:);
yaw2=yaw2(n:nend);
rd2=rd2(n:nend)*pi/180;
vcxy_hat2=vcxy_hat2(n:nend,:);
ug2=ug2(n:nend);
t=t(n:nend);
e1=pd1-p1;
e2=pd2-p2;

k=length(e1(:,1));
for i=1:k
    k3=0.08;
V1(i)=sqrt(x1(i,1)^2+x1(i,2)^2+x1(i,3)^2);
V2(i)=sqrt(x2(i,1)^2+x2(i,2)^2+x2(i,3)^2);
    
%     norme1(i)=norm(e1(i,:));
%     norme2(i)=norm(e2(i,:));
end
%%  Plot mission
    fig1=figure(1)
    plot(pd1(:,2),pd1(:,1),'k');
    hold on;
%     plot(pd2(:,2),pd2(:,1),'k');
%     legend('show')
%     legend('Desired path','MPCPF','NPF');legend('Location','northwest');legend('boxoff');
     plot(p1(:,2),p1(:,1),'b-.','LineWidth',1);
     plot(p2(:,2),p2(:,1),'r-.','LineWidth',1);
    set(gca,'Color',[0.98 0.98 .98]);
    title('Trajectories of the Medusa vehicle');
    xlabel('Y[m]');
    ylabel('X[m]');
%     axis([-15 15 -5 25]);
    axis equal;
   


%% Plot heading
%     figure(2)
%     for i=1:n
%         if yaw1(i)>250 && (yaw1(i)<=360)
%             yaw1(i)=yaw1(i)-360;
%         end
%         if yaw2(i)>250 && (yaw2(i)<=360)
%             yaw2(i)=yaw2(i)-360;
%         end
% %         if yaw3(i)>250 && (yaw3(i)<=360)
% %             yaw3(i)=yaw3(i)-360;
% %         end
%     end
    fig2=figure(2) ;
%     subplot(2,1,1);
%     plot(t,yaw1,'k');
%     hold on
%     plot(t,yaw2,'r');
% %     plot(t,yaw3(k:n),'color',ycol);
%     grid;
%     title('Heading');
%     ylabel('yaw[degree]');
%     xlabel('t[second]');
%     legend('show')
%     legend('MedBlack','MedRed');legend('Location','north');legend('boxoff');
%     tmin=t(1)-1; tmax=t(end)+1;
%     limit=[tmin tmax -10 370];
%     axis(limit);
% 
%     subplot(2,1,2);
    plot(t,V1,'b');
    hold on;
    plot(t,V2,'r');
    title('Path following error');
    ylabel('||\textbf{e})(t)||','Interpreter','latex');
    xlabel('$$t[second]$$','Interpreter','latex');
%     legend('show')
     legend('MPCPF','NPF');
     grid on;
%bbbbblegend('Location','northeast');legend('boxoff');
    tmin=t(1)-1; tmax=t(end)+1;
    limit=[tmin tmax -inf inf];
    axis(limit);
    grid;
    
    
    
%% Patch vehicle to the mission    
    resample_3vehicles;
    animation_3vehicles_LMH;    % animation_3vehicles_LMH; for circle
    
%     str = ' $$ v_{cx}=0m/s$$ \\ $$v_{cy}=0m/s$$' ;
%     text(-45,30, str,'Interpreter','latex')
%     
    dim = [0.2 0.5 0.3 0.3];
    str = {'$$v_{cx}=0.2m/s$$','$$v_{cy}=0.2m/s$$'};
    annotation('textbox',dim,'String',str,'Interpreter','latex','FitBoxToText','on');
    
    %% Plot evolution of rabit
    fig3= figure(3);
    subplot(2,1,1);
    plot(t,ug1,'b');hold on; plot(t,ug2,'r');
    tmin=t(1)-1; tmax=t(end)+1;
    limit=[tmin tmax -1 1];
    axis(limit);
    title('Speed of the rabit');
%     ylabel('sdot[m/s]');
    ylabel('$$\dot{s}[m/s]$$','Interpreter','latex');
    xlabel('$$t[second]$$','Interpreter','latex');
%     legend('show')
    legend('MPCPF','NPF');
%    legend('Location','northeast');legend('boxoff');
    plot(t,ug_max,'k--');plot(t,ug_min,'k--');
    grid on;
    
    subplot(2,1,2);
    plot(t,rd1,'b');hold on; plot(t,rd2,'r');
    tmin=t(1)-1; tmax=t(end)+1;
    limit=[tmin tmax -.4 .4];
    axis(limit);
    title('Heading rate');
    ylabel('$$r_{d}[rad/s]$$','Interpreter','latex');
    xlabel('$$t[second]$$','Interpreter','latex');
%     legend('show')
    legend('MPCPF','NPF');
%    legend('Location','northeast');legend('boxoff');
    plot(t,rd_max,'k--');plot(t,rd_min,'k--');
    grid on;
%      
%     saveas(fig1,'mission_circlecurrent.png');
%     saveas(fig2,'heading_circlecurrent.png');
%     saveas(fig3,'ug_circlecurrent.png');
% 
% 
%     
%     
%     
%     
%     
%     
%     
%     
%     saveas(fig1,'mission_circlecurrent.png');
%     saveas(fig2,'heading_circlecurrent.png');
%     saveas(fig3,'ug_circlecurrent.png');

