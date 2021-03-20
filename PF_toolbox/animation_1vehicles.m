%% Video
vid=0;
v = VideoWriter('heart');
v.FrameRate=5;
open(v);

fig1=figure(1);
%fig2=figure(2);
%colors
 
AUV_COL= [0 0 0];
 

AUV_COL11= [0.8 0.8 0];
AUV_COL22= [0.1 0.1 0];
AUV_COL33= [0.9 0.2 0];
 
 
% yaw2_22=90-yaw22*180/pi;
% yaw3_33=90-yaw33*180/pi;

% Com1_old=COM1.Data(1);
% Com2_old=COM2.Data(1);
% Com3_old=COM3.Data(1);
 set (fig1, 'Units', 'normalized', 'Position', [0,0,1,1]);
 pd=x_path(:,1:2);
 p=x_robot(:,1:2);
yaw=x_robot(:,3)*180/pi;
%yaw=90-yaw;

for i=1:10:size(pd,1)%data.i,
    
    hold off
    figure(fig1);
    % path black
h1 =  plot(pd(1:end,2),pd(1:end,1),'--','LineWidth',0.2,'Color','b');
    hold on; 
%     plot(pd22(1:i,2),pd22(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
%     plot(pd33(1:i,2),pd33(1:i,1),'-','LineWidth',0.2,'Color',[200,10,10]/255);
    
 h2 =   plot(pd(i,2),pd(i,1),'r--*','MarkerSize',20);
%     T=x_path(i,9:10);
%     N=x_path(i,11:12);
%     arrow3([pd(i,2),pd(i,1)],[pd(i,2)+5*T(2),pd(i,1)+5*T(1)]);
%     arrow3([pd(i,2),pd(i,1)],[pd(i,2)+5*N(2),pd(i,1)+5*N(1)]);
 h3 =  plot(p(1:i,2),p(1:i,1),'.-','LineWidth',0.5,'Color',[10,10,0]/255);
   
%     plot(p22(1:i,2),p22(1:i,1),'-','LineWidth',0.2,'Color',[10,10,0]/255);
%     plot(p33(1:i,2),p33(1:i,1),'-','LineWidth',0.2,'Color',[200,10,10]/255);
    hold on;
    %view(45,20)    
    % path myellow
   Scale=.2;
 
    GTF_Simulink_PlotAUV([p(i,2),p(i,1),0], [0,0,90-yaw(i)], Scale, 0,AUV_COL,1);  
   
%     GTF_Simulink_PlotAUV([p22(i,2),p22(i,1),0], [0,0,yaw2_22(i)], Scale, 0,AUV_COL22,1);  
%     GTF_Simulink_PlotAUV([p33(i,2),p33(i,1),0], [0,0,yaw3_33(i)], Scale, 0,AUV_COL33,1);  
 legend([h1,h2,h3],'Desired path', 'Reference point to track','Vehicle trajectory');   
if (i==100)
    hello=0;
end

    Scale=.25*8;
    grid on; %axis equal;
%    axis([-15 15 -15 15 ])
   
    title('Animation of path following of autonomous vehicles')
    xlabel('Y[m]')
    ylabel('X[m]')
    axis equal;
    if(vid==1)
        drawnow;
        currFrame = getframe(fig1);
        writeVideo(v, currFrame);
    else
        pause(0);
        drawnow;
    end
    hold off;
 %   pause(0.5);
    
%     figure(fig2)
%     subplot(3,1,1);hold on;
%     if COM1.Data(i)~=Com1_old
%         stem(COM1.Time(i),1,'Color',[0.8,0.9,0]);
%         title('transmistion signal on vehicle 1');
%         Com1_old=COM1.Data(i);
%     end
%     subplot(3,1,2);hold on;
%     if COM2.Data(i)~=Com2_old
%         stem(COM2.Time(i),1,'k')
%         title('transmistion signal on vehicle 2');
%         Com2_old=COM2.Data(i);
%     end
%     subplot(3,1,3);hold on;
%     if COM3.Data(i)~=Com3_old
%         stem(COM3.Time(i),1,'r')
%         title('transmistion signal on vehicle 3')
%         Com3_old=COM3.Data(i);
%     end 
end

if(vid==1)
    close(v);
%     close(fig);
end