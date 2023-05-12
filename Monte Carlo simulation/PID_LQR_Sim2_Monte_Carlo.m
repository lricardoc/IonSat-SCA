%%Script to run multiple Monte Carlo simulations during the week-end

%% PID Sim2 : Robustness to the thruster activation
clear
close

%%Initialize parameters				
%Load IONSat parameters
load('SatConstant_Updated_04-2023.mat')
%Configure the RW that failed 
RW_failure_matrix = ones(1,4); % no failed RW; if 0 failed RW if 1 functionning RW

%Initial sat inertia
sat_inertia_original = sat.inertia;
sat_inertia = sat.inertia;

%Fix the simulation parameters 
TimeStep = 1;        %fixed-step size in solver, Default time step=0.25
Torbit=2*pi*sqrt((orbit.a)^3/(3.986004418E5));  %(one orbit is ~5400 s)
N_orbits = 6;           %number of orbits to be simulated
%Time spent performing the simulation in seconds :
t_sim = Torbit*N_orbits;
% t_sim = 3000;


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

%%Disturbance block parameters
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

          %number of thrust firings

%%Monte Carlo simulation
n = 300; % number of simulation, take around 12 minutes for n=10

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

PID_initial_date = zeros(3,n);

PID_rotation_thrust_deviation = zeros(1,n);

PID_Thrust_application_point = zeros(3,n);

PID_RAAN_ArgumentPeriapsis_TrueAnomaly = zeros(3,n);



for i = 1:n
    % Add noise to initial conditions
    %Randomize IONSat's inertia at -+20% of the fixed inertia
    %Configuration of the Earth's magnetic field model
    date.month = round(12*rand);
    date.day = round(28*rand);
    date_IGRF = [date.year,date.month,date.day];
    
    %Thruster direction and activation:
    sat.thruster.force=0.00075; % Force of thruster in [N]
    d=1*pi/180;           %deviation in rads of the thruster wrt to x axis in plane XY
    
    psi=rand*360*pi/180;       %rotation of thrust deviation around x axis
    
        R=[ 1    0            0;...
            0    cos(psi)  -sin(psi);...
            0    sin(psi)   cos(psi)];
    sat.thruster.dir=R*[cos(d);sin(d);0];   %Direction of thrust (unit vector)
    
    %Thruster applied point in BRF:
    x_force_thruster= 182 + 2*rand;       %Distance from the geometric center x axis in [mm]
    y_force_thruster= -1 + 2*rand;      %Distance from the geometric center y axis in [mm]
    z_force_thruster= -1 +2*rand;      %Distance from the geometric center z axis in [mm]
    
    sat.thruster.point=[-x_force_thruster;y_force_thruster;z_force_thruster]/1e3;    
                                %Distance from the geometric center (BRF) in [m]
                                %Thruster located in "back side" of S/C (-X)
    %other parameters for thruster firing
    sat.thruster.duration=3000;         %duration of the thrust in [seconds]
    sat.thruster.wait=7000;             %waiting time between thrusts in [s]
    sat.thruster.firstimpulse=4000;     %first thrust after start sim in [s]
                                        %needs to be less than waiting time
    sat.thruster.Nfirings=3;  

    %orbit altitude generate randomly between 
    %orbit.a_noisy = orbit.a + rand*100;      
    %Orbit: Initialisation of Keplerian parameters
%     orbit.a = 6678 + rand*200;     %semimajor axis [km]

    orbit.a = 6378+300; %semi-marjor axis km
    orbit.e = 0.001;    %eccentricity
    orbit.i = 98;     %inclination [degrees]
    orbit.O = 360*rand;      %Right ascension of the right ascending node [degrees] %max 197, min 300.5 %181
    orbit.o = 360*rand;      %Argument of the perigee [degrees]                      %max 90, min 0      %90
    orbit.nu = orbit.o - orbit.O;       %True anomaly [degrees]
    
    MODE_manager;

%     % generate random number between -180 and +180     
%     att.alpha = 360*rand - 180;   % Yaw  
%     att.beta = 360*rand - 180;     % Pitch   
%     att.gamma = 360*rand - 180;    % Roll   
% 
    att.alpha = 5;   % Yaw  
    att.beta = -5;     % Pitch   
    att.gamma = 5;    % Roll   

    %Initial angular velocities in each axis (x,y,z) of body frame [degrees/sec]
%     att.wx0 = 40*rand - 20;    % generate random number between -20 and +20    
%     att.wy0 = 40*rand - 20;
%     att.wz0 = 40*rand - 20;
%     
    att.wx0 = 0;    
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

    PID_RW1_speed_command(:,i) = simResults.RW_speed_command.Data(1,:);
    PID_RW2_speed_command(:,i) = simResults.RW_speed_command.Data(2,:);
    PID_RW3_speed_command(:,i) = simResults.RW_speed_command.Data(3,:);
    PID_RW4_speed_command(:,i) = simResults.RW_speed_command.Data(4,:);

    PID_RW1_control_torque(:,i) = simResults.RW_control_torque.Data(1,:);
    PID_RW2_control_torque(:,i) = simResults.RW_control_torque.Data(2,:);
    PID_RW3_control_torque(:,i) = simResults.RW_control_torque.Data(3,:);
    PID_RW4_control_torque(:,i) = simResults.RW_control_torque.Data(4,:);

    PID_RW_saturation_duration(:,i) = simResults.RW_saturation_time.Data(n_points,:);
    
    PID_initial_date(:,i) = transpose(date_IGRF);

    PID_rotation_thrust_deviation(i) = psi*180/pi;

    PID_Thrust_application_point(:,i) = transpose(sat.thruster.point);

    PID_RAAN_ArgumentPeriapsis_TrueAnomaly(:,i) = [orbit.O; orbit.o; orbit.nu];
end

%%Data processing
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

save('PID_Monte_Carlo_Sim2_Thrust_activation_300km_Norbit6_n300.mat');



%% LQR
%Clear all the variables except for the RW failure matrix
clearvars -except RW_failure_matrix

%%Initialize parameters				
%Load IONSat parameters
load('SatConstant_Updated_04-2023.mat')

sat_inertia_original = sat.inertia;
sat_inertia = sat.inertia;
%Fix the simulation parameters 
TimeStep = 1;        %fixed-step size in solver, Default time step=0.25
Torbit=2*pi*sqrt((orbit.a)^3/(3.986004418E5));  %(one orbit is ~5400 s)
N_orbits = 6;           %number of orbits to be simulated
%Time spent performing the simulation in seconds :
t_sim = Torbit*N_orbits;
% t_sim = 3000;


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

%%Disturbance block parameters
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


%%LQR controller feedback gain
%If the Inertia is fixed choose between the following two feedback gain matrix
%Keep in mind that there might be other good performing parameters

% K = [0.0013  0  0  0.0111  0  0;
%     0  0.0016  0  0  0.0129  0;
%     0  0  0.0017  0  0  0.0157]; %q=0.004 r=1000

K = [0.00053417  0  0  0.0070506  0  0;
    0  0.00071733 0  0  0.00840481  0;
    0  0  0.00081350 0  0  0.010476]; %q=0.004 r=4000


%%Monte Carlo simulation
n = 300; % number of simulation, it take between 95 and 120 minutes for n=100

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

LQR_initial_date = zeros(3,n);

LQR_rotation_thrust_deviation = zeros(1,n);

LQR_Thrust_application_point = zeros(3,n);

LQR_RAAN_ArgumentPeriapsis_TrueAnomaly = zeros(3,n);

for i = 1:n
    % Add noise to initial conditions
    date.month = round(12*rand);
    date.day = round(28*rand);
    date_IGRF = [date.year,date.month,date.day];
    
    %Thruster direction and activation:
    sat.thruster.force=0.00075; % Force of thruster in [N]
    d=1*pi/180;           %deviation in rads of the thruster wrt to x axis in plane XY
    
    psi=rand*360*pi/180;       %rotation of thrust deviation around x axis
    
        R=[ 1    0            0;...
            0    cos(psi)  -sin(psi);...
            0    sin(psi)   cos(psi)];
    sat.thruster.dir=R*[cos(d);sin(d);0];   %Direction of thrust (unit vector)
    
    %Thruster applied point in BRF:
    x_force_thruster= 182 + 2*rand;       %Distance from the geometric center x axis in [mm]
    y_force_thruster= -1 + 2*rand;      %Distance from the geometric center y axis in [mm]
    z_force_thruster= -1 +2*rand;      %Distance from the geometric center z axis in [mm]
    
    sat.thruster.point=[-x_force_thruster;y_force_thruster;z_force_thruster]/1e3;    
                                %Distance from the geometric center (BRF) in [m]
                                %Thruster located in "back side" of S/C (-X)
    %other parameters for thruster firing
    sat.thruster.duration=3000;         %duration of the thrust in [seconds]
    sat.thruster.wait=7000;             %waiting time between thrusts in [s]
    sat.thruster.firstimpulse=4000;     %first thrust after start sim in [s]
                                        %needs to be less than waiting time
    sat.thruster.Nfirings=3;  

    %orbit altitude generate randomly between 
    %orbit.a_noisy = orbit.a + rand*100;      
    %Orbit: Initialisation of Keplerian parameters
%     orbit.a = 6678 + rand*200;     %semimajor axis [km]

    orbit.a = 6378+300; %semi-marjor axis km
    orbit.e = 0.001;    %eccentricity
    orbit.i = 98;     %inclination [degrees]
    orbit.O = 360*rand;      %Right ascension of the right ascending node [degrees] %max 197, min 300.5 %181
    orbit.o = 360*rand;      %Argument of the perigee [degrees]                      %max 90, min 0      %90
    orbit.nu = orbit.o - orbit.O;       %True anomaly [degrees]

    MODE_manager;

    % generate random number between -180 and +180     
%     att.alpha = 360*rand - 180;   % Yaw  
%     att.beta = 360*rand - 180;     % Pitch   
%     att.gamma = 360*rand - 180;    % Roll   
% 
    att.alpha = 5;   % Yaw  
    att.beta = -5;     % Pitch   
    att.gamma = 5;    % Roll   

%     %Initial angular velocities in each axis (x,y,z) of body frame [degrees/sec]
%     att.wx0 = 40*rand - 20;    % generate random number between -5 and +5    
%     att.wy0 = 40*rand - 20;
%     att.wz0 = 40*rand - 20;
    
    att.wx0 = 0;    % generate random number between -5 and +5    
    att.wy0 = 0;
    att.wz0 = 0;

    %Save the inital conditions
    Alpha0(i) = att.alpha;
    Beta0(i) = att.beta;
    Gamma0(i) = att.gamma;
    WX0(i) = att.wx0;
    WY0(i) = att.wy0;
    WZ0(i) = att.wz0;
    N(i)=i;



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
    
    LQR_initial_date(:,i) = transpose(date_IGRF);

    LQR_rotation_thrust_deviation(i) = psi*180/pi;

    LQR_Thrust_application_point(:,i) = transpose(sat.thruster.point);

    LQR_RAAN_ArgumentPeriapsis_TrueAnomaly(:,i) = [orbit.O; orbit.o; orbit.nu];
end

%%Data processing
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

save('LQR_Monte_Carlo_Sim2_Thrust_activation_300km_Norbit6_n300.mat');


%% Simulation at 270km and n=50; same conditions for the other parameters
PID_LQR_Sim2_Monte_Carlo_270km_n50;
