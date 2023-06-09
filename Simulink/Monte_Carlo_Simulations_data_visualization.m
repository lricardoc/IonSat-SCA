%% Script to plot data from Monte Carlo simulations

%% Plot the output of the simulations

% Plot initial angle rates
figure;
tiledlayout(2,1);
% Plot the initial angle for each simulations
nexttile;
hold on;
scatter(N,Alpha0);
scatter(N,Beta0);
scatter(N,Gamma0);
legend('alpha','beta','gamma')
xlabel('n-th simulation');
ylabel('initial angle (in °)');
title('Initial angle');
% Plot the initial angular velocities for each simulations
nexttile;
hold on;
scatter(N,WX0);
scatter(N,WY0);
scatter(N,WZ0);
legend('Wx0','Wy0','Wz0')
xlabel('n-th simulation');
ylabel('initial angular velocities (in °/s)');
title('Initial angular velocities');


%% Plot the angle error rate
%angle error rate (real vs ref)
figure;
t = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(PID_angle_error_rate_x(:,j));
end
title('Roll angle');
nexttile;
hold on;
for j = 1:n
    plot(PID_angle_error_rate_y(:,j));
end
title('Pitch angle');
nexttile;
hold on;
for j = 1:n
    plot(PID_angle_error_rate_z(:,j));
end
title('Yaw angle');
title(t,'Angle error rate of the CubeSat')
xlabel(t,'Time in seconds')
ylabel(t,'Error rate (in °)')

%Comparing real atitude with the estimated attitude
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
title(t,'Comparison of the real attitude and the estimated attitude')
xlabel(t,'Time in seconds')
ylabel(t,'Difference (in °)')


%Comparing real atitude with the reference attitude
figure;
t = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(Attitude_real_vs_reference_x(:,j));
end
title('Roll angle');
nexttile;
hold on;
for j = 1:n
    plot(Attitude_real_vs_reference_y(:,j));
end
title('Pitch angle');
nexttile;
hold on;
for j = 1:n
    plot(Attitude_real_vs_reference_z(:,j));
end
title('Yaw angle');
title(t,'Comparison of the real attitude and the reference attitude')
xlabel(t,'Time in seconds')
ylabel(t,'Difference (in °)')

%Comparing estimated atitude with the reference attitude
figure;
t = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(Attitude_estimated_vs_reference_x(:,j));
end
title('Roll angle');
nexttile;
hold on;
for j = 1:n
    plot(Attitude_estimated_vs_reference_y(:,j));
end
title('Pitch angle');
nexttile;
hold on;
for j = 1:n
    plot(Attitude_estimated_vs_reference_z(:,j));
end
title('Yaw angle');
title(t,'Comparison of the reference attitude and the estimated attitude')
xlabel(t,'Time in seconds')
ylabel(t,'Difference (in °)')


%% plot the angular velocities error rate
figure;
a = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(PID_angular_velocity_error_rate_x(:,j));
end
title('x axis');
nexttile;
hold on;
for j = 1:n
    plot(PID_angular_velocity_error_rate_y(:,j));
end
title('y axis');
nexttile;
hold on;
for j = 1:n
    plot(PID_angular_velocity_error_rate_z(:,j));
end
title('z axis');
xlabel(a,'Time in seconds');
ylabel(a,'Angular velocity error rate (in °/s)');
title(a,'Angular velocity error rate of the CubeSat')

% comparing angular velocities real vs estimated
figure;
a = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(Angular_velocity_real_vs_estimated_x(:,j));
end
title('x axis');
nexttile;
hold on;
for j = 1:n
    plot(Angular_velocity_real_vs_estimated_y(:,j));
end
title('y axis');
nexttile;
hold on;
for j = 1:n
    plot(Angular_velocity_real_vs_estimated_z(:,j));
end
title('z axis');
xlabel(a,'Time in seconds');
ylabel(a,'Angular velocity error rate (in °/s)');
title(a,'Angular velocity difference between the real and the estimated state')


% comparing angular velocities real vs reference
figure;
a = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(Angular_velocity_real_vs_reference_x(:,j));
end
title('x axis');
nexttile;
hold on;
for j = 1:n
    plot(Angular_velocity_real_vs_reference_y(:,j));
end
title('y axis');
nexttile;
hold on;
for j = 1:n
    plot(Angular_velocity_real_vs_reference_z(:,j));
end
title('z axis');
xlabel(a,'Time in seconds');
ylabel(a,'Angular velocity error rate (in °/s)');
title(a,'Angular velocity difference between the real and the reference state')


% comparing angular velocities reference vs estimated
figure;
a = tiledlayout(3,1);
nexttile;
hold on;
for j = 1:n
    plot(Angular_velocity_estimated_vs_reference_x(:,j));
end
title('x axis');
nexttile;
hold on;
for j = 1:n
    plot(Angular_velocity_estimated_vs_reference_y(:,j));
end
title('y axis');
nexttile;
hold on;
for j = 1:n
    plot(Angular_velocity_estimated_vs_reference_z(:,j));
end
title('z axis');
xlabel(a,'Time in seconds');
ylabel(a,'Angular velocity error rate (in °/s)');
title(a,'Angular velocity difference betwwen the real and the estimated state')



%% Plot the average magnitude of the angles
figure;
tiledlayout(2,1);
nexttile;
%plot the average magnitude of the angle error
hold on;
scatter(N,PID_average_magnitude_angle_error(:,1));
scatter(N,PID_average_magnitude_angle_error(:,2));
scatter(N,PID_average_magnitude_angle_error(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angle error (in °)');
title('Average magnitude of the angle error');
%plot the average magnitude of the angular velocities error
nexttile;
hold on;
scatter(N,PID_average_magnitude_angular_velocities_error(:,1));
scatter(N,PID_average_magnitude_angular_velocities_error(:,2));
scatter(N,PID_average_magnitude_angular_velocities_error(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angular velocities error (in °/s)');
title('Average magnitude of the angular velocities error');


%average magnitude real vs estimated
figure;
tiledlayout(2,1);
nexttile;
%plot the average magnitude of the angle error
hold on;
scatter(N,Attitude_average_mangnitude_real_vs_estimated(:,1));
scatter(N,Attitude_average_mangnitude_real_vs_estimated(:,2));
scatter(N,Attitude_average_mangnitude_real_vs_estimated(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angle error (in °)');
title('Average magnitude of the difference between the real and the estimated attitude');
%plot the average magnitude of the angular velocities error
nexttile;
hold on;
scatter(N,Angular_velocity_average_mangnitude_real_vs_estimated(:,1));
scatter(N,Angular_velocity_average_mangnitude_real_vs_estimated(:,2));
scatter(N,Angular_velocity_average_mangnitude_real_vs_estimated(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angular velocities error (in °/s)');
title('Average magnitude of the difference between the real and the estimated angular velocities');


%average magnitude real vs estimated
figure;
tiledlayout(2,1);
nexttile;
%plot the average magnitude of the angle error
hold on;
scatter(N,Attitude_average_mangnitude_real_vs_reference(:,1));
scatter(N,Attitude_average_mangnitude_real_vs_reference(:,2));
scatter(N,Attitude_average_mangnitude_real_vs_reference(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angle error (in °)');
title('Average magnitude of the difference between the real and the reference attitude');
%plot the average magnitude of the angular velocities error
nexttile;
hold on;
scatter(N,Angular_velocity_average_mangnitude_real_vs_reference(:,1));
scatter(N,Angular_velocity_average_mangnitude_real_vs_reference(:,2));
scatter(N,Angular_velocity_average_mangnitude_real_vs_reference(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angular velocities error (in °/s)');
title('Average magnitude of the difference between the real and the reference angular velocities');


%average magnitude reference vs estimated
figure;
tiledlayout(2,1);
nexttile;
%plot the average magnitude of the angle error
hold on;
scatter(N,Attitude_average_mangnitude_estimated_vs_reference(:,1));
scatter(N,Attitude_average_mangnitude_estimated_vs_reference(:,2));
scatter(N,Attitude_average_mangnitude_estimated_vs_reference(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angle error (in °)');
title('Average magnitude of the difference between the estimated and the reference attitude');
%plot the average magnitude of the angular velocities error
nexttile;
hold on;
scatter(N,Angular_velocity_average_mangnitude_estimated_vs_reference(:,1));
scatter(N,Angular_velocity_average_mangnitude_estimated_vs_reference(:,2));
scatter(N,Angular_velocity_average_mangnitude_estimated_vs_reference(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angular velocities error (in °/s)');
title('Average magnitude of the difference between the estimated and the reference angular velocities');



%% Plot RW related information
%plot the average power consumption of all the RW
figure;
hold on;
scatter(N,PID_RW_average_power_consumtion);
xlabel('n-th simulation');
ylabel('Power consumption (in W)');
title('Average power consumption of all the RW');

%Plot the RW saturation duration
figure;
hold on;
scatter(N,PID_RW_saturation_duration(1,:));
scatter(N,PID_RW_saturation_duration(2,:));
scatter(N,PID_RW_saturation_duration(3,:));
scatter(N,PID_RW_saturation_duration(4,:));
legend('RW1','RW2','RW3','RW4');
xlabel('n-th simulation');
ylabel('Saturation duration (in seconds)');
title('RW saturation duration');

%plot the RW speed command
figure
a = tiledlayout(4,1);
nexttile;
hold on;
for j = 1:n
    plot(PID_RW1_speed_command(:,j));
end
title('RW1');
nexttile;
hold on;
for j = 1:n
    plot(PID_RW2_speed_command(:,j));
end
title('RW2');
nexttile;
hold on;
for j = 1:n
    plot(PID_RW3_speed_command(:,j));
end
title('RW3');
nexttile;
hold on;
for j = 1:n
    plot(PID_RW4_speed_command(:,j));
end
title('RW4');
xlabel(a,'Time in seconds')
ylabel(a,'Speed in RPM')
title(a,'RW speed command')


%plot the RW control torque
figure
a = tiledlayout(4,1);
nexttile;
hold on;
for j = 1:n
    plot(PID_RW1_control_torque(:,j));
end
title('RW1');
nexttile;
hold on;
for j = 1:n
    plot(PID_RW2_control_torque(:,j));
end
title('RW2');
nexttile;
hold on;
for j = 1:n
    plot(PID_RW3_control_torque(:,j));
end
title('RW3');
nexttile;
hold on;
for j = 1:n
    plot(PID_RW4_control_torque(:,j));
end
title('RW4');
xlabel(a,'Time in seconds')
ylabel(a,'Torque in Nm')
title(a,'RW control torque')


%Plot the inertia 
figure;
hold on;
scatter(N,Inertia(1,:));
scatter(N,Inertia(2,:));
scatter(N,Inertia(3,:));
scatter(N,Inertia(4,:));
scatter(N,Inertia(5,:));
scatter(N,Inertia(6,:));
legend('I_{xx}','I_{yy}','I_{zz}','I_{xy}','I_{xz}','I_{yz}');
xlabel('n-th simulation');
ylabel('Inertia in kg.m²');
title('Inertia of IONSat for every simulations');

