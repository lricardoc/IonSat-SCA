%% Plot the output of the simulations
 N = zeros(100,1);
 for i = 1:100
    N(i) =i;
 end


% Plot the angle error rate
figure;
t = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(MEKF_angle_error_rate_x(:,j));
end
title('Roll angle');
nexttile;
hold on;
for j = 1:n
    plot(MEKF_angle_error_rate_y(:,j));
end
title('Pitch angle');
nexttile;
hold on;
for j = 1:n
    plot(MEKF_angle_error_rate_z(:,j));
end
title('Yaw angle');
title(t,'Angle error rate of the CubeSat')
xlabel(t,'Time in seconds')
ylabel(t,'Error rate (in °)')



% Plot the angle estimation rate
% n_point = length(Attitude_real_vs_estimated_x(:,1));
% Attitude_real_vs_estimated_process_x = zeros(n_point,n);
% Attitude_real_vs_estimated_process_y = zeros(n_point,n);
% Attitude_real_vs_estimated_process_z = zeros(n_point,n);
% for i=1:100
%     parfor j=1:n_point
%         if Attitude_real_vs_estimated_x(j,i)>180 
%             Attitude_real_vs_estimated_process_x(j,i)=mod(360,Attitude_real_vs_estimated_x(j,i));
%         elseif Attitude_real_vs_estimated_x(j,i)<-180
%             Attitude_real_vs_estimated_process_x(j,i)=mod(-360,Attitude_real_vs_estimated_x(j,i));
%         else
%             Attitude_real_vs_estimated_process_x(j,i)=Attitude_real_vs_estimated_x(j,i);
%         end    
%         if Attitude_real_vs_estimated_y(j,i)>180 
%             Attitude_real_vs_estimated_process_y(j,i)=mod(360,Attitude_real_vs_estimated_y(j,i));
%         elseif Attitude_real_vs_estimated_y(j,i)<-180
%             Attitude_real_vs_estimated_process_y(j,i)=mod(-360,Attitude_real_vs_estimated_y(j,i));
%         else
%             Attitude_real_vs_estimated_process_y(j,i)=Attitude_real_vs_estimated_y(j,i);
%         end    
%         if Attitude_real_vs_estimated_z(j,i)>180 
%             Attitude_real_vs_estimated_process_z(j,i)=mod(360,Attitude_real_vs_estimated_z(j,i));
%         elseif Attitude_real_vs_estimated_z(j,i)<-180
%             Attitude_real_vs_estimated_process_z(j,i)=mod(-360,Attitude_real_vs_estimated_z(j,i));
%         else
%             Attitude_real_vs_estimated_process_z(j,i)=Attitude_real_vs_estimated_z(j,i);
%         end
%     end
% end
figure;
t = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(Attitude_real_vs_estimated_x(:,j));
end
title('Roll angle');
nexttile;
hold on;
for j = 1:n
    plot(Attitude_real_vs_estimated_y(:,j));
end
title('Pitch angle');
nexttile;
hold on;
for j = 1:n
    plot(Attitude_real_vs_estimated_z(:,j));
end
title('Yaw angle');
title(t,'Angle estimation error rate of the CubeSat')
xlabel(t,'Time in seconds')
ylabel(t,'Error rate (in °)')


figure;
t = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(Attitude_real_vs_estimated_process_x(:,j));
end
title('Roll angle');
nexttile;
hold on;
for j = 1:n
    plot(Attitude_real_vs_estimated_process_y(:,j));
end
title('Pitch angle');
nexttile;
hold on;
for j = 1:n
    plot(Attitude_real_vs_estimated_process_z(:,j));
end
title('Yaw angle');
title(t,'Processed angle estimation error of the CubeSat')
xlabel(t,'Time in seconds')
ylabel(t,'Processed error (in °)')





%plot the angular velocities error rate
figure;
a = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(MEKF_angular_velocity_error_rate_x(:,j));
end
title('x axis');
nexttile;
hold on;
for j = 1:n
    plot(MEKF_angular_velocity_error_rate_y(:,j));
end
title('y axis');
nexttile;
hold on;
for j = 1:n
    plot(MEKF_angular_velocity_error_rate_z(:,j));
end
title('z axis');
xlabel(a,'Time in seconds');
ylabel(a,'Angular velocity error rate (in °/s)');
title(a,'Angular velocity error rate of the CubeSat')



%plot the gyro bias
figure;
a = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(MEKF_bias_x(:,j));
end
title('x axis');
nexttile;
hold on;
for j = 1:n
    plot(MEKF_bias_y(:,j));
end
title('y axis');
nexttile;
hold on;
for j = 1:n
    plot(MEKF_bias_z(:,j));
end
title('z axis');
xlabel(a,'Time in seconds');
ylabel(a,'Gyroscope bias (in °/s)');
title(a,'Gyroscope bias')



%Plot the average magnitude of the angles
figure;
tiledlayout(4,1);
nexttile;
%plot the average magnitude of the angle error
hold on;
scatter(N,MEKF_average_magnitude_angle_error(:,1));
scatter(N,MEKF_average_magnitude_angle_error(:,2));
scatter(N,MEKF_average_magnitude_angle_error(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angle error (in °)');
title('Average magnitude of the angle error');

%plot the average magnitude of the angle estimation error
nexttile;
hold on;
scatter(N,Attitude_average_mangnitude_real_vs_estimated_process(:,1));
scatter(N,Attitude_average_mangnitude_real_vs_estimated_process(:,2));
scatter(N,Attitude_average_mangnitude_real_vs_estimated_process(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angle error (in °)');
title('Average magnitude of the angle estimation error');
%plot the average magnitude of the angular velocities error
nexttile;
hold on;
scatter(N,MEKF_average_magnitude_angular_velocities_error(:,1));
scatter(N,MEKF_average_magnitude_angular_velocities_error(:,2));
scatter(N,MEKF_average_magnitude_angular_velocities_error(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angular velocities error (in °/s)');
title('Average magnitude of the angular velocities error');
%plot the average magnitude of the biais
nexttile;
hold on;
scatter(N,MEKF_average_magnitude_bias(:,1));
scatter(N,MEKF_average_magnitude_bias(:,2));
scatter(N,MEKF_average_magnitude_bias(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Bias ');
title('Average magnitude of the gyroscope bias');


% comparing the gyro bias with the angular velocity error rate
figure;
scatter(N,rad2deg(MEKF_average_magnitude_bias(:,1))-MEKF_average_magnitude_angular_velocities_error(:,1));
scatter(N,rad2deg(MEKF_average_magnitude_bias(:,2))-MEKF_average_magnitude_angular_velocities_error(:,2));
scatter(N,rad2deg(MEKF_average_magnitude_bias(:,3))-MEKF_average_magnitude_angular_velocities_error(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('deg/s ');
title('Comparing the average gyroscope bias to angular velocity error rate')


%plot the difference between the angular velocities error rate and the gyro bias
figure;
a = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(MEKF_angular_velocity_error_rate_x(:,j)-rad2deg(MEKF_bias_x(:,j)));
end
title('x axis');
nexttile;
hold on;
for j = 1:n
    plot(MEKF_angular_velocity_error_rate_y(:,j)-rad2deg(MEKF_bias_y(:,j)));
end
title('y axis');
nexttile;
hold on;
for j = 1:n
    plot(MEKF_angular_velocity_error_rate_z(:,j)-rad2deg(MEKF_bias_z(:,j)));
end
title('z axis');
xlabel(a,'Time in seconds');
ylabel(a,'deg/s');
title(a,'Difference between the angular velocities error rate and the gyro bias')