%% Plot the values to compare
% Script that plot the errors to comapre the eficiency
% Position error
figure(10);
subplot(3,2,1);
plot(gola(:,1),'Color','g');
hold on
plot(END(:,1),'Color','r');
legend("X Robot","X Human");
hold off
title('X');
% xlabel('Index');
ylabel('Position X (m)');

subplot(3,2,2);
plot(END(:,2),'Color','g');
hold on

plot(gola(:,2),'Color','r');
legend("Y Robot","Y Human");
hold off
title('Y');
% xlabel('Index');
ylabel('Position Y (m)');

subplot(3,2,3);
plot(gola(:,3),'Color','g');
hold on
plot(END(:,3),'Color','r');
legend("Z Robot","Z Human");
hold off
title('Z');
% xlabel('Index');
ylabel('Position Z (m)');

subplot(3,2,4);
boxplot(abs([gola(:,:) - END(:,:)]),'Labels',{'Error X','Error Y','Error Z'});
title('Position Error');
ylabel('Error (m)');


%% Orientation error
THETA = [];
THETA2 = [];
V = [];
V2 = [];
for i=1:3:length(ROTEND)
    R1 = [ROTMAT(i,1),ROTMAT(i,2),ROTMAT(i,3);ROTMAT(i+1,1),ROTMAT(i+1,2),ROTMAT(i+1,3);ROTMAT(i+2,1),ROTMAT(i+2,2),ROTMAT(i+2,3)];
    R2 = [ROTEND(i,1),ROTEND(i,2),ROTEND(i,3);ROTEND(i+1,1),ROTEND(i+1,2),ROTEND(i+1,3);ROTEND(i+2,1),ROTEND(i+2,2),ROTEND(i+2,3)];
    R_1 = R1;
    R_2 = R2;
    ang_axis = rotm2axang(R_1); % Guarda la salida de la función en una variable
    ang_axis2 = rotm2axang(R_2);
    theta = ang_axis(4); % Ángulo de rotación en radianes
    theta2 = ang_axis2(4);
    v = ang_axis(1:3); % Coordenadas del eje de rotación unitario
    v2 = ang_axis2(1:3);
    THETA = [THETA;theta];
    THETA2 = [THETA2;theta2];
    V = [V;v];
    V2 = [V2;v2];
    
end

subplot(3,2,5);
plot(THETA);
hold on
plot(THETA2)
title('Rotation end-effector');
% xlabel('Index');
ylabel('Theta (rad)');
legend("Theta Robot","Theta Human");
hold off

subplot(3,2,6);
boxplot(abs(THETA - THETA2)*0.3);
title('Error Theta');
ylabel('Theta error (rad)');
hold off
