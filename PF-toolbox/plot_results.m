%% Video
vid = 1;
v = VideoWriter('all_methods');
v.FrameRate=5;
open(v);

fig1=figure(1);
%fig2=figure(2);
%colors
 
% AUV_COL= [0 0 0];
 

AUV_COL(:,:,1) = [0.8 0.8 0];
AUV_COL(:,:,2) = [0.1 0.1 0];
AUV_COL(:,:,3) = [0.9 0.2 0];

AUV_COL(:,:,4) = [0.6 0.8 0];
AUV_COL(:,:,5) = [0.9 0.1 0.5];
AUV_COL(:,:,6) = [0.9 0.8 0]; 
AUV_COL(:,:,7) = [0.9 0.2 .9]; 
 
% yaw2_22=90-yaw22*180/pi;
% yaw3_33=90-yaw33*180/pi;

% Com1_old=COM1.Data(1);
% Com2_old=COM2.Data(1);
% Com3_old=COM3.Data(1);
 set (fig1, 'Units', 'normalized', 'Position', [0,0,1,1]);
for i = 1:7
 X_path(:,:,i) = eval(strcat('x_path',num2str(i)));
 X_robot(:,:,i)  = eval(strcat('x_robot',num2str(i)));
 end

 Scale=.5;

for i=1:10:size(pd1,1)%data.i,
    
    hold off
    figure(fig1);
    title('Path following methods');
    for j = 1:8
            subplot(2,4,j);
            hold off;
       if (j <= 7)
            % path black
            plot(X_path(1:end,2,j),X_path(1:end,1,j),'--','LineWidth',1,'Color','b');
            hold on; 
            plot(X_path(i,2,j),X_path(i,1,j),'r*','MarkerSize',20, 'LineWidth', 2);
            plot(X_robot(1:i,2,j),X_robot(1:i,1,j),'-','LineWidth',1.5,'Color',[10,10,0]/255);
            GTF_Simulink_PlotAUV([X_robot(i,2,j),X_robot(i,1,j),0], [0,0, 90-X_robot(i,3,j)*180/pi], Scale, 0,AUV_COL(:,:,j),1);  

            axis([-18, 15 -22 30]);
            str = ['Method',' ', num2str(j)]; 
            title1 =  title(str);
            title1.FontSize = 12;
            xlabel('Y[m]');
            ylabel('X[m]');
            box on;
%             grid on;

%            subplot(2,4,j);
       else
            h11 =  plot(X_path(1,2,7),X_path(1,1,7),'--','LineWidth',1,'Color','b');
            hold on; 
            h21 =  plot(X_path(1,2,7),X_path(1,1,7),'r*','MarkerSize',20, 'LineWidth', 2);
            h31 =  plot(X_robot(1,2,7),X_robot(1,1,7),'-','LineWidth',1.5,'Color',[10,10,0]/255);
            led = legend([h11,h21,h31],'Desired path', '"Reference point" on the path','Vehicle trajectory','Location','east');   
            led.FontSize = 12;
            set(gca,'XTick',[]);
            set(gca,'YTick',[]);
            axis([100, 101, 100, 101]);

       end
         
         
    end

%     subplot(2,4,2);
%     hold off;
%     h12 =  plot(pd2(1:end,1),pd2(1:end,2),'--','LineWidth',1,'Color','b');
%     hold on;
%     h22 =   plot(pd2(i,1),pd2(i,2),'r*','MarkerSize',20, 'LineWidth', 2);
%     h32 =  plot(p2(1:i,1),p2(1:i,2),'-','LineWidth',1.5,'Color',[10,10,0]/255);
%     GTF_Simulink_PlotAUV([p2(i,1),p2(i,2),0], [0,0, yaw2(i)], Scale, 0,AUV_COL,1);  
%     
%     subplot(2,4,3);
%     hold off;
%     
%     h13 =  plot(pd3(1:end,1),pd3(1:end,2),'--','LineWidth',1,'Color','b');
%      hold on;
%     h23 =   plot(pd3(i,1),pd3(i,2),'r*','MarkerSize',20, 'LineWidth', 2);
%     h33 =  plot(p3(1:i,1),p3(1:i,2),'-','LineWidth',1.5,'Color',[10,10,0]/255);
%     GTF_Simulink_PlotAUV([p3(i,1),p3(i,2),0], [0,0, yaw3(i)], Scale, 0,AUV_COL,1);  
%  
%      grid on; %axis equal;

   if(vid==1)
        drawnow;
        currFrame = getframe(fig1);
        writeVideo(v, currFrame);
    else
        pause(0);
        drawnow;
    end
    hold off;
    
end

if(vid==1)
    close(v);
%     close(fig);
end