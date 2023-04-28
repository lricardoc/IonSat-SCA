%% Initialize parameters

PID_Monte_Carlo_initial_conditions;

%% Monte Carlo simulation
n = 10; % number of simulation 

%initialize lists of data
n_points = round(t_sim) + 1; % number of points to save per simulations

Alpha0 = zeros(1,n);
Beta0 = zeros(1,n);
Gamma0 = zeros(1,n);
WX0 = zeros(1,n);
WY0 = zeros(1,n);
WZ0 = zeros(1,n);
N = zeros(1,n);

PID_angle_error_rate_x = zeros(n_points,n);
PID_angle_error_rate_y = zeros(n_points,n);
PID_angle_error_rate_z = zeros(n_points,n);

PID_angular_velocity_error_rate_x = zeros(n_points,n);
PID_angular_velocity_error_rate_y = zeros(n_points,n);
PID_angular_velocity_error_rate_z = zeros(n_points,n);

PID_total_power_consumption = zeros(n_points,n);

PID_RW1_speed_command = zeros(n_points,n);
PID_RW2_speed_command = zeros(n_points,n);
PID_RW3_speed_command = zeros(n_points,n);
PID_RW4_speed_command = zeros(n_points,n);

PID_RW1_control_torque = zeros(n_points,n);
PID_RW2_control_torque = zeros(n_points,n);
PID_RW3_control_torque = zeros(n_points,n);
PID_RW4_control_torque = zeros(n_points,n);

PID_RW_saturation_duration = zeros(4,n);

for i = 1:n
    % Add noise to initial conditions

    %Randomize IONSat's inertia at -+10% of the fixed inertia
%     I_xx =  0.0702*9/10 + 0.0702*rand/5;
%     I_yy =  0.113*9/10 + 0.113*rand/5;
%     I_zz =  0.16*9/10 + 0.16*rand/5;
%     I_xy =  0.0017*9/10 + 0.0017*rand/5;
%     I_xz = -0.0023*9/10 + 0.0023*rand/5;
%     I_yz = -0.0003*9/10 + 0.0003*rand/5;
%     sat.inertia = [I_xx I_xy I_xz;
%                    I_xy I_yy I_yz;
%                    I_xz I_yz I_zz];

      
    %Orbit: Initialisation of Keplerian parameters
    orbit.a = 6678 + rand*200;     %semimajor axis [km], random value between 6678 and 6878 km
    orbit.e = 0.001;    %eccentricity
    orbit.i = 98;     %inclination [degrees]
    orbit.O = 10;      %Right ascension of the right ascending node [degrees] %max 197, min 300.5 %181
    orbit.o = 90;      %Argument of the perigee [degrees]                      %max 90, min 0      %90
    orbit.nu = 0;       %True anomaly [degrees]
    
    MODE_manager; % Compute the initial quaternion according to the MODE

    % generate random number between -180 and +180     
    att.alpha = 360*rand - 180;   % Yaw  
    att.beta = 360*rand - 180;     % Pitch   
    att.gamma = 360*rand - 180;    % Roll     
        
    %Initial angular velocities in each axis (x,y,z) of body frame [degrees/sec]
    att.wx0 = 10*rand - 5;    % generate random number between -5 and +5    
    att.wy0 = 10*rand - 5;
    att.wz0 = 10*rand - 5;
    
    %Save the inital conditions
    Alpha0(i) = att.alpha;
    Beta0(i) = att.beta;
    Gamma0(i) = att.gamma;
    WX0(i) = att.wx0;
    WY0(i) = att.wy0;
    WZ0(i) = att.wz0;
    N(i)=i;

    % run the simulink 
    simResults = sim('Control_v7_PID_for_Monte_Carlo.slx');

    %save the data
    PID_angle_error_rate_x(:,i) = simResults.angle_error_rate.Data(:,1);
    PID_angle_error_rate_y(:,i) = simResults.angle_error_rate.Data(:,2);
    PID_angle_error_rate_z(:, i) = simResults.angle_error_rate.Data(:,3);

    PID_angular_velocity_error_rate_x(:,i) = simResults.angular_velocities_error_rate.Data(1,:);
    PID_angular_velocity_error_rate_y(:,i) = simResults.angular_velocities_error_rate.Data(2,:);
    PID_angular_velocity_error_rate_z(:,i) = simResults.angular_velocities_error_rate.Data(3,:);

    PID_total_power_consumption(:,i) = simResults.RW_total_power_consumption.Data(:);

    PID_RW1_speed_command(:,i) = simResults.RW_speed_command.Data(:,1);
    PID_RW2_speed_command(:,i) = simResults.RW_speed_command.Data(:,2);
    PID_RW3_speed_command(:,i) = simResults.RW_speed_command.Data(:,3);
    PID_RW4_speed_command(:,i) = simResults.RW_speed_command.Data(:,4);

    PID_RW1_control_torque(:,i) = simResults.RW_control_torque.Data(1,:);
    PID_RW2_control_torque(:,i) = simResults.RW_control_torque.Data(2,:);
    PID_RW3_control_torque(:,i) = simResults.RW_control_torque.Data(3,:);
    PID_RW4_control_torque(:,i) = simResults.RW_control_torque.Data(4,:);

    PID_RW_saturation_duration(:,i) = simResults.RW_saturation_time.Data(n_points,:);
end

%% Data processing
PID_average_magnitude_angle_error = zeros(n,3);

PID_average_magnitude_angular_velocities_error = zeros(n,3);

PID_RW_average_power_consumtion = zeros(1,n);

for k =1:n
    %Compute the average magnitude (RMS) of the angle error
    PID_average_magnitude_angle_error(k,1) = sqrt(mean(PID_angle_error_rate_x(:,k).^2 ));
    PID_average_magnitude_angle_error(k,2) = sqrt(mean(PID_angle_error_rate_y(:,k).^2 ));
    PID_average_magnitude_angle_error(k,3) = sqrt(mean(PID_angle_error_rate_z(:,k).^2 ));

    %Compute the average magnitude (RMS) of the angular velocities error
    PID_average_magnitude_angular_velocities_error(k,1) = sqrt(mean(PID_angular_velocity_error_rate_x(:,k).^2 ));
    PID_average_magnitude_angular_velocities_error(k,2) = sqrt(mean(PID_angular_velocity_error_rate_y(:,k).^2 ));
    PID_average_magnitude_angular_velocities_error(k,3) = sqrt(mean(PID_angular_velocity_error_rate_z(:,k).^2 ));

    % compute the average totalt power consumption of the RW
    PID_RW_average_power_consumtion(k) = mean(PID_total_power_consumption(:,k));
end

%% Plot data
PID_Monte_Carlo_plot_data;