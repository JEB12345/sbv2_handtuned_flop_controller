%% Transformation matrices

load('transformations.mat') % Training data

%% Hebi Stuff

hebiStuff;

%% Go to initial position

% Loop Zeros so not to pull too much current went resetting robot
Cmd.position = ones(1,24)*NaN;
for j=1:24
    Cmd.position(j) = 0.0;
    Group.send(Cmd);
    disp(j);
    pause(0.1);
end

%%
a1 = [0; 0; 0; 0];
a2 = [0; 0; 0; 0];
a3 = [0; 0; 0; 0];
a4 = [0; 0; 0; 0];
a_fused = [0;0;0;0];


g1 = [0; 0; 0; 0];
g2 = [0; 0; 0; 0];
g3 = [0; 0; 0; 0];
g4 = [0; 0; 0; 0];
fused = [0;0;0;0];

t = 1;
figure();
hold on;

while(1)

    a0_1 = a1;
    a0_2 = a2;
    a0_3 = a3;
    a0_4 = a4;
    a_fused0 = a_fused;
    
    g0_1 = g1;
    g0_2 = g2;
    g0_3 = g3;
    g0_4 = g4;
    fused0 = fused;
    
    
    tmpFeedback = Group.getNextFeedback;
    
    accel1 = [tmpFeedback.accelX(1); tmpFeedback.accelY(1); tmpFeedback.accelZ(1); 0];
    accel2 = [tmpFeedback.accelX(2); tmpFeedback.accelY(2); tmpFeedback.accelZ(2); 0];
    accel3 = [tmpFeedback.accelX(3); tmpFeedback.accelY(3); tmpFeedback.accelZ(3); 0];
    accel4 = [tmpFeedback.accelX(4); tmpFeedback.accelY(4); tmpFeedback.accelZ(4); 0];
    
    
    gyro1 = [tmpFeedback.gyroX(1); tmpFeedback.gyroY(1); tmpFeedback.gyroZ(1); 0];
    gyro2 = [tmpFeedback.gyroX(2); tmpFeedback.gyroY(2); tmpFeedback.gyroZ(2); 0];
    gyro3 = [tmpFeedback.gyroX(3); tmpFeedback.gyroY(3); tmpFeedback.gyroZ(3); 0];
    gyro4 = [tmpFeedback.gyroX(4); tmpFeedback.gyroY(4); tmpFeedback.gyroZ(4); 0];
    
    a1 = T_b1m1*accel1;
    a2 = T_b1m2*accel2;
    a3 = T_b1m3*accel3;
    a4 = T_b1m4*accel4;
    
    g1 = T_b1m1*gyro1;
    g2 = T_b1m2*gyro2;
    g3 = T_b1m3*gyro3;
    g4 = T_b1m4*gyro4;
    
    
%     plot([t-1 t],[2+g0_1(1) 2+g1(1)],'-ko','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','k')
%     plot([t-1 t],[g0_1(2) g1(2)],'-kx','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','k')
%     plot([t-1 t],[-2+g0_1(3) -2+g1(3)],'-k*','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','k')
%     
%     plot([t-1 t],[2+g0_2(1) 2+g2(1)],'-ro','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','r')
%     plot([t-1 t],[g0_2(2) g2(2)],'-rx','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','r')
%     plot([t-1 t],[-2+g0_2(3) -2+g2(3)],'-r*','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','r')
% 
%     plot([t-1 t],[2+g0_3(1) 2+g3(1)],'-go','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','g')
%     plot([t-1 t],[g0_3(2) g3(2)],'-gx','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','g')
%     plot([t-1 t],[-2+g0_3(3) -2+g3(3)],'-g*','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','g')
%     
%     plot([t-1 t],[2+g0_4(1) 2+g4(1)],'-bo','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','b')
%     plot([t-1 t],[g0_4(2) g4(2)],'-bx','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','b')
%     plot([t-1 t],[-2+g0_4(3) -2+g4(3)],'-b*','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','b')
%     
%     fused = [mean([g1(1) g2(1) g3(1) g4(1)]);mean([g1(2) g2(2) g3(2) g4(2)]);mean([g1(3) g2(3) g3(3) g4(3)]);1];
%     
%     plot([t-1 t],[2+fused0(1) 2+fused(1)],'-yo','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','y')
%     plot([t-1 t],[fused0(2) fused(2)],'-yx','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','y')
%     plot([t-1 t],[-2+fused0(3) -2+fused(3)],'-y*','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','y')
%     
    
%     plot([t-1 t],[a0_1(1) a1(1)],'-ko','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','k')
%     plot([t-1 t],[a0_1(2) a1(2)],'-kx','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','k')
%     plot([t-1 t],[a0_1(3) a1(3)],'-k*','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','k')
%     
%     plot([t-1 t],[a0_2(1) a2(1)],'-ro','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','r')
%     plot([t-1 t],[a0_2(2) a2(2)],'-rx','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','r')
%     plot([t-1 t],[a0_2(3) a2(3)],'-r*','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','r')
% 
%     plot([t-1 t],[a0_3(1) a3(1)],'-go','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','g')
%     plot([t-1 t],[a0_3(2) a3(2)],'-gx','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','g')
%     plot([t-1 t],[a0_3(3) a3(3)],'-g*','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','k','MarkerFaceColor','g')
%     
%     plot([t-1 t],[a0_4(1) a4(1)],'-bo','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','b')
%     plot([t-1 t],[a0_4(2) a4(2)],'-bx','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','b')
%     plot([t-1 t],[a0_4(3) a4(3)],'-b*','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','b')
%     
    a_fused = [mean([a1(1) a2(1) a3(1) a4(1)]);mean([a1(2) a2(2) a3(2) a4(2)]);mean([a1(3) a2(3) a3(3) a4(3)]);1];
    
    plot([t-1 t],[a_fused0(1) a_fused(1)],'-yo','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','y')
    plot([t-1 t],[a_fused0(2) a_fused(2)],'-yx','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','y')
    plot([t-1 t],[a_fused0(3) a_fused(3)],'-y*','LineWidth',1,'MarkerSize',4,'MarkerEdgeColor','b','MarkerFaceColor','y')
   


    t = t+1;
    xlim([t-100 t+10])
    
    pause(0.01);

end