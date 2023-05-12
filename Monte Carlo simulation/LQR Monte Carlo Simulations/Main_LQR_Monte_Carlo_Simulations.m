clear
close
%% Initialize parameters				
%Load IONSat parameters
load('SatConstant_Updated_04-2023.mat')

RW_failure_matrix = [1 0 1 1];
    
sat_inertia_original = sat.inertia;

%Fix the simulation parameters 
TimeStep = 1;        %fixed-step size in solver, Default time step=0.25
Torbit=2*pi*sqrt((orbit.a)^3/(3.986004418E5));  %(one orbit is ~5400 s)
N_orbits = 1;           %number of orbits to be simulated
%Time spent performing the simulation in seconds :
t_sim = Torbit*N_orbits;
% t_sim = 3000;

%Configuration of the Earth's magnetic field model
date_IGRF = [date.year,date.month,date.day];
%Determine the order of approximation for IGRF model of the earth's magnetic field nmax (must be equal or less than 13)
nmax = 2;
%Determine whether you are using full IGRF model or dipole approximation; 1 for full model, 0 for dipole approximation
Use_IGRF = 1; 


%Configuration of the Reaction Wheels 
alpha = 26.6 * pi/180;      %from deg to rad
sat.wheel.repartition_matrix_4RW = [[cos(alpha),      0,    -cos(alpha),      0    ];...
                                    [     0,     cos(alpha),     0,     -cos(alpha)];...
                                    [sin(alpha), sin(alpha), sin(alpha), sin(alpha)]];
sat.wheel.repartition_metrix_4RW_inverse = pinv(sat.wheel.repartition_matrix_4RW); %pseudo inverse matrix
%% Disturbance block parameters
%%% Model of Perturbation Torques
%Magnetic Torque Residual dipole
sat.residual_dipole=[0.08;0.08;0.08];  %residual magnetic dipole in [A*m^2]

%Aerodynamic Torque Constants
load('IonSat_6U.mat');
IonSataero.T = SNAP_aeromodel.T;
IonSataero.El = SNAP_aeromodel.Az;
IonSataero.Az = SNAP_aeromodel.El;
IonSataero.Tx = SNAP_aeromodel.T_x;
IonSataero.Ty = SNAP_aeromodel.T_y;
IonSataero.Tz = SNAP_aeromodel.T_z;
IonSataero.av_density_vs_alt = SNAP_aeromodel.av_density_vs_alt;
IonSataero.alt_range = SNAP_aeromodel.alt_range;

%Thruster direction and activation:
sat.thruster.force=0.00075; % Force of thruster in [N]
d=1*pi/180;           %deviation in rads of the thruster wrt to x axis in plane XY
alpha=80*pi/180;       %rotation of thrust deviation around x axis
    R=[ 1    0            0;...
        0    cos(alpha)  -sin(alpha);...
        0    sin(alpha)   cos(alpha)];
sat.thruster.dir=R*[cos(d);sin(d);0];   %Direction of thrust (unit vector)
%Thruster applied point in BRF:
x_force_thruster=182.95;       %Distance from the geometric center x axis in [mm]
y_force_thruster=0;      %Distance from the geometric center y axis in [mm]
z_force_thruster=0;      %Distance from the geometric center z axis in [mm]
sat.thruster.point=[-x_force_thruster;y_force_thruster;z_force_thruster]/1e3;    
                            %Distance from the geometric center (BRF) in [m]
                            %Thruster located in "back side" of S/C (-X)
%other parameters for thruster firing
sat.thruster.duration=3000;         %duration of the thrust in [seconds]
sat.thruster.wait=7000;             %waiting time between thrusts in [s]
sat.thruster.firstimpulse=4000;     %first thrust after start sim in [s]
                                    %needs to be less than waiting time
sat.thruster.Nfirings=3;            %number of thrust firings

%% LQR controller feedback gain
%If the Inertia is fixed choose between the following two feedback gain matrix
%Keep in mind that there might be other good performing parameters

% K = [0.0013  0  0  0.0111  0  0;
%     0  0.0016  0  0  0.0129  0;
%     0  0  0.0017  0  0  0.0157]; %q=0.004 r=1000

K = [0.00053417  0  0  0.0070506  0  0;
    0  0.00071733 0  0  0.00840481  0;
    0  0  0.00081350 0  0  0.010476]; %q=0.004 r=4000


%% Monte Carlo simulation
n = 3; % number of simulation, it take between 95 and 120 minutes for n=100

%initialize lists of data
n_points = round(t_sim) + 1; % number of points to save per simulations

Alpha0 = zeros(1,n);
Beta0 = zeros(1,n);
Gamma0 = zeros(1,n);
WX0 = zeros(1,n);
WY0 = zeros(1,n);
WZ0 = zeros(1,n);
N = zeros(1,n);

LQR_angle_error_rate_x = zeros(n_points,n);
LQR_angle_error_rate_y = zeros(n_points,n);
LQR_angle_error_rate_z = zeros(n_points,n);

LQR_angular_velocity_error_rate_x = zeros(n_points,n);
LQR_angular_velocity_error_rate_y = zeros(n_points,n);
LQR_angular_velocity_error_rate_z = zeros(n_points,n);

LQR_total_power_consumption = zeros(n_points,n);

LQR_RW1_speed_command = zeros(n_points,n);
LQR_RW2_speed_command = zeros(n_points,n);
LQR_RW3_speed_command = zeros(n_points,n);
LQR_RW4_speed_command = zeros(n_points,n);

LQR_RW1_control_torque = zeros(n_points,n);
LQR_RW2_control_torque = zeros(n_points,n);
LQR_RW3_control_torque = zeros(n_points,n);
LQR_RW4_control_torque = zeros(n_points,n);

LQR_RW_saturation_duration = zeros(4,n);

Orbit_altitude = zeros(1,n);

Inertia =  zeros(6,n);
Feedback_gain = zeros(6,n);

for i = 1:n
    % Add noise to initial conditions
    %Randomize IONSat's inertia at -+20% of the fixed inertia
    % If the Inertia is fixed for all n simulations, comment the following 9 lines
    I_xx =  0.0702*9/10 + 0.0702*rand/5;
    I_yy =  0.113*9/10 + 0.113*rand/5;
    I_zz =  0.16*9/10 + 0.16*rand/5;
    I_xy =  0.0017*9/10 + 0.0017*rand/5;
    I_xz = -0.0023*9/10 + 0.0023*rand/5;
    I_yz = -0.0003*9/10 + 0.0003*rand/5;
    sat_inertia = [I_xx I_xy I_xz;
                   I_xy I_yy I_yz;
                   I_xz I_yz I_zz];
    %Compute the new feedback gain matrix
    %NB:The feedback gain depend of the inertia, 
    %therefore if the inertia changes the feedback gain has to be updated accordingly 
    %LQR_Compute_feedback_gain_matrix; % If the Inertia is fixed for all n simulations, comment this line

    %orbit altitude generate randomly between 
    %orbit.a_noisy = orbit.a + rand*100;      
    %Orbit: Initialisation of Keplerian parameters
%     orbit.a = 6678 + rand*200;     %semimajor axis [km]

    orbit.a = 6378+300;

    orbit.e = 0.001;    %eccentricity
    orbit.i = 98;     %inclination [degrees]
    orbit.O = 10;      %Right ascension of the right ascending node [degrees] %max 197, min 300.5 %181
    orbit.o = 90;      %Argument of the perigee [degrees]                      %max 90, min 0      %90
    orbit.nu = 0;       %True anomaly [degrees]
    
    MODE_manager;

    % generate random number between -180 and +180     
%     att.alpha = 360*rand - 180;   % Yaw  
%     att.beta = 360*rand - 180;     % Pitch   
%     att.gamma = 360*rand - 180;    % Roll   
% 
    att.alpha = 5;   % Yaw  
    att.beta = -5;     % Pitch   
    att.gamma = 5;    % Roll   

    %Initial angular velocities in each axis (x,y,z) of body frame [degrees/sec]
%     att.wx0 = 40*rand - 20;    % generate random number between -5 and +5    
%     att.wy0 = 40*rand - 20;
%     att.wz0 = 40*rand - 20;
%     
    att.wx0 = 0;    % generate random number between -5 and +5    
    att.wy0 = -0;
    att.wz0 = 0;

    %Save the inital conditions
    Alpha0(i) = att.alpha;
    Beta0(i) = att.beta;
    Gamma0(i) = att.gamma;
    WX0(i) = att.wx0;
    WY0(i) = att.wy0;
    WZ0(i) = att.wz0;
    N(i)=i;
    Orbit_altitude(i) = orbit.a - 6378;
    
    Inertia(1,i) = I_xx;
    Inertia(2,i) = I_yy;
    Inertia(3,i) = I_zz;
    Inertia(4,i) = I_xy;
    Inertia(5,i) = I_xz;
    Inertia(6,i) = I_yz;
    
    Feedback_gain(1,i) = K(1);
    Feedback_gain(2,i) = K(5);
    Feedback_gain(3,i) = K(9);
    Feedback_gain(4,i) = K(10);
    Feedback_gain(5,i) = K(14);
    Feedback_gain(6,i) = K(18);


    % run the simulink 
    simResults = sim('Control_v6_LQR_2020b_for_Monte_carlo.slx');

    %save the data
    LQR_angle_error_rate_x(:,i) = simResults.angle_error_rate.Data(:,1);
    LQR_angle_error_rate_y(:,i) = simResults.angle_error_rate.Data(:,2);
    LQR_angle_error_rate_z(:, i) = simResults.angle_error_rate.Data(:,3);

    LQR_angular_velocity_error_rate_x(:,i) = simResults.angular_velocities_error_rate.Data(1,:);
    LQR_angular_velocity_error_rate_y(:,i) = simResults.angular_velocities_error_rate.Data(2,:);
    LQR_angular_velocity_error_rate_z(:,i) = simResults.angular_velocities_error_rate.Data(3,:);

    LQR_total_power_consumption(:,i) = simResults.RW_total_power_consumption.Data(:);

    LQR_RW1_speed_command(:,i) = simResults.RW_speed_command.Data(1,:);
    LQR_RW2_speed_command(:,i) = simResults.RW_speed_command.Data(2,:);
    LQR_RW3_speed_command(:,i) = simResults.RW_speed_command.Data(3,:);
    LQR_RW4_speed_command(:,i) = simResults.RW_speed_command.Data(4,:);

    LQR_RW1_control_torque(:,i) = simResults.RW_control_torque.Data(:,1);
    LQR_RW2_control_torque(:,i) = simResults.RW_control_torque.Data(:,2);
    LQR_RW3_control_torque(:,i) = simResults.RW_control_torque.Data(:,3);
    LQR_RW4_control_torque(:,i) = simResults.RW_control_torque.Data(:,4);

    LQR_RW_saturation_duration(:,i) = simResults.RW_saturation_time.Data(n_points,:);
end

%% Data processing
LQR_average_magnitude_angle_error = zeros(n,3);

LQR_average_magnitude_angular_velocities_error = zeros(n,3);

LQR_RW_average_power_consumtion = zeros(1,n);

for k =1:n
    %Compute the average magnitude (RMS) of the angle error
    LQR_average_magnitude_angle_error(k,1) = sqrt(mean(LQR_angle_error_rate_x(:,k).^2 ));
    LQR_average_magnitude_angle_error(k,2) = sqrt(mean(LQR_angle_error_rate_y(:,k).^2 ));
    LQR_average_magnitude_angle_error(k,3) = sqrt(mean(LQR_angle_error_rate_z(:,k).^2 ));

    %Compute the average magnitude (RMS) of the angular velocities error
    LQR_average_magnitude_angular_velocities_error(k,1) = sqrt(mean(LQR_angular_velocity_error_rate_x(:,k).^2 ));
    LQR_average_magnitude_angular_velocities_error(k,2) = sqrt(mean(LQR_angular_velocity_error_rate_y(:,k).^2 ));
    LQR_average_magnitude_angular_velocities_error(k,3) = sqrt(mean(LQR_angular_velocity_error_rate_z(:,k).^2 ));

    % compute the average totalt power consumption of the RW
    LQR_RW_average_power_consumtion(k) = mean(LQR_total_power_consumption(:,k));
end


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
    plot(LQR_angle_error_rate_x(:,j));
end
title('Roll angle');
nexttile;
hold on;
for j = 1:n
    plot(LQR_angle_error_rate_y(:,j));
end
title('Pitch angle');
nexttile;
hold on;
for j = 1:n
    plot(LQR_angle_error_rate_z(:,j));
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
    plot(LQR_angular_velocity_error_rate_x(:,j));
end
title('x axis');
nexttile;
hold on;
for j = 1:n
    plot(LQR_angular_velocity_error_rate_y(:,j));
end
title('y axis');
nexttile;
hold on;
for j = 1:n
    plot(LQR_angular_velocity_error_rate_z(:,j));
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
scatter(N,LQR_average_magnitude_angle_error(:,1));
scatter(N,LQR_average_magnitude_angle_error(:,2));
scatter(N,LQR_average_magnitude_angle_error(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angle error (in °)');
title('Average magnitude of the angle error');
%plot the average magnitude of the angular velocities error
nexttile;
hold on;
scatter(N,LQR_average_magnitude_angular_velocities_error(:,1));
scatter(N,LQR_average_magnitude_angular_velocities_error(:,2));
scatter(N,LQR_average_magnitude_angular_velocities_error(:,3));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angular velocities error (in °/s)');
title('Average magnitude of the angular velocities error');


%plot the average power consumption of all the RW
figure;
hold on;
scatter(N,LQR_RW_average_power_consumtion);
xlabel('n-th simulation');
ylabel('Power consumption (in W)');
title('Average power consumption of all the RW');

%Plot the RW saturation duration
figure;
hold on;
scatter(N,LQR_RW_saturation_duration(1,:));
scatter(N,LQR_RW_saturation_duration(2,:));
scatter(N,LQR_RW_saturation_duration(3,:));
scatter(N,LQR_RW_saturation_duration(4,:));
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
    plot(LQR_RW1_speed_command(:,j));
end
title('RW1');
nexttile;
hold on;
for j = 1:n
    plot(LQR_RW2_speed_command(:,j));
end
title('RW2');
nexttile;
hold on;
for j = 1:n
    plot(LQR_RW3_speed_command(:,j));
end
title('RW3');
nexttile;
hold on;
for j = 1:n
    plot(LQR_RW4_speed_command(:,j));
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
    plot(LQR_RW1_control_torque(:,j));
end
title('RW1');
nexttile;
hold on;
for j = 1:n
    plot(LQR_RW2_control_torque(:,j));
end
title('RW2');
nexttile;
hold on;
for j = 1:n
    plot(LQR_RW3_control_torque(:,j));
end
title('RW3');
nexttile;
hold on;
for j = 1:n
    plot(LQR_RW4_control_torque(:,j));
end
title('RW4');
xlabel(a,'Time in seconds')
ylabel(a,'Torque in Nm')
title(a,'RW control torque')
% 
% %Plot the orbit altitude
% figure;
% hold on;
% scatter(N,Orbit_altitude);
% xlabel('n-th simulation');
% ylabel('Altitude in km');
% title('Orbit altitude');
% 
% %Plot the inertia 
% figure;
% hold on;
% scatter(N,Inertia(1,:));
% scatter(N,Inertia(2,:));
% scatter(N,Inertia(3,:));
% scatter(N,Inertia(4,:));
% scatter(N,Inertia(5,:));
% scatter(N,Inertia(6,:));
% legend('I_{xx}','I_{yy}','I_{zz}','I_{xy}','I_{xz}','I_{yz}');
% xlabel('n-th simulation');
% ylabel('Inertia in kg.m²');
% title('Inertia of IONSat for every simulations');
% 
% %Plot the feedback gain coefficients 
% figure;
% hold on;
% scatter(N,Feedback_gain(1,:));
% scatter(N,Feedback_gain(2,:));
% scatter(N,Feedback_gain(3,:));
% scatter(N,Feedback_gain(4,:));
% scatter(N,Feedback_gain(5,:));
% scatter(N,Feedback_gain(6,:));
% 
% 
% legend('Kq1','Kq2','Kq3','Kw1','Kw2','Kw3');
% xlabel('n-th simulation');
% ylabel('Feedback gain value');
% title('Feedback gain relevant coefficients');