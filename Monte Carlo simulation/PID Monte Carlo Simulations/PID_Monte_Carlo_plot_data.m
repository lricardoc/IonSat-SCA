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


% Plot the angle error rate
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


%plot the angular velocities error rate
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


%Plot the average magnitude of the angles
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
scatter(N,PID_RW_saturation_duration);
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

%Plot the orbit altitude
figure;
hold on;
scatter(N,Orbit_altitude);
xlabel('n-th simulation');
ylabel('Altitude in km');
title('Orbit altitude');

%Plot the inertia 
figure;
hold on;
scatter(N,Inertia);
legend('I_{xx}','I_{yy}','I_{zz}','I_{xy}','I_{xz}','I_{yz}');
xlabel('n-th simulation');
ylabel('Inertia in kg.m²');
title('Inertia of IONSat for every simulations');

