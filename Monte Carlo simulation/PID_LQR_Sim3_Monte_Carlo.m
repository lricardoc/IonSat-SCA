%% PID SIM3 : robustness to the changind modes

clear
close
%Initialize parameters				
%Load IONSat parameters
load('SatConstant_Updated_04-2023.mat')

RW_failure_matrix = ones(1,4); % no failed RW; if 0 failed RW if 1 functionning RW


%Initial sat inertia
sat_inertia_original = sat.inertia;
sat_inertia = sat.inertia;

%Fix the simulation parameters 
TimeStep = 1;        %fixed-step size in solver, Default time step=0.25
Torbit=2*pi*sqrt((orbit.a)^3/(3.986004418E5));  %(one orbit is ~5400 s)
N_orbits = 9;           %number of orbits to be simulated
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
%Disturbance block parameters
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


%Monte Carlo simulation
n = 180; % number of simulation, take around 12 minutes for n=10

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

Mode_sequence = zeros(3,n);

for i = 1:n
    % Add noise to initial conditions

    %Orbit: Initialisation of Keplerian parameters
    orbit.a = 6378+300;
    orbit.e = 0.001;    %eccentricity
    orbit.i = 98;     %inclination [degrees]
    orbit.O = 10;      %Right ascension of the right ascending node [degrees] %max 197, min 300.5 %181
    orbit.o = 90;      %Argument of the perigee [degrees]                      %max 90, min 0      %90
    orbit.nu = 0;       %True anomaly [degrees]
    
    
    MODE_SEQUENCE = randperm(5,3); %generate 3 diferrent radom values between 1 and 5; their are 5!/(5-3)!= 60 possible combinations

    MODE = MODE_SEQUENCE(1);
    MODE_manager_bis;

    % generate random number between -180 and +180     
    att.alpha = 360*rand - 180;   % Yaw  
    att.beta = 360*rand - 180;     % Pitch   
    att.gamma = 360*rand - 180;    % Roll   


    %Initial angular velocities in each axis (x,y,z) of body frame [degrees/sec]
    att.wx0 = 40*rand - 20;    % generate random number between -20 and +20    
    att.wy0 = 40*rand - 20;
    att.wz0 = 40*rand - 20;

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
    
    Mode_sequence(:,i) = transpose(MODE_SEQUENCE);
end

%Data processing
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

save('PID_Sim3_changing_x3_modes_9orbits_n180.mat');

%% LQR
%Clear all the previous variables exceep for the mode sequence
clearvars -except Mode_sequence

%Initialize parameters				
%Load IONSat parameters
load('SatConstant_Updated_04-2023.mat')

sat_inertia_original = sat.inertia;
sat_inertia = sat.inertia;

%Fix the simulation parameters 
TimeStep = 1;        %fixed-step size in solver, Default time step=0.25
Torbit=2*pi*sqrt((orbit.a)^3/(3.986004418E5));  %(one orbit is ~5400 s)
N_orbits = 9;           %number of orbits to be simulated
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
%Disturbance block parameters
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

%LQR controller feedback gain
%If the Inertia is fixed choose between the following two feedback gain matrix
%Keep in mind that there might be other good performing parameters

% K = [0.0013  0  0  0.0111  0  0;
%     0  0.0016  0  0  0.0129  0;
%     0  0  0.0017  0  0  0.0157]; %q=0.004 r=1000

K = [0.00053417  0  0  0.0070506  0  0;
    0  0.00071733 0  0  0.00840481  0;
    0  0  0.00081350 0  0  0.010476]; %q=0.004 r=4000


%Monte Carlo simulation
n = 180; % number of simulation, it take between 95 and 120 minutes for n=100

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



for i = 1:n
    % Add noise to initial conditions
      
    %Orbit: Initialisation of Keplerian parameters
    orbit.a = 6378+300;
    orbit.e = 0.001;    %eccentricity
    orbit.i = 98;     %inclination [degrees]
    orbit.O = 10;      %Right ascension of the right ascending node [degrees] %max 197, min 300.5 %181
    orbit.o = 90;      %Argument of the perigee [degrees]                      %max 90, min 0      %90
    orbit.nu = 0;       %True anomaly [degrees]
    
    MODE_SEQUENCE = Mode_sequence(:,i); %generate 3 diferrent radom values between 1 and 5

    MODE = MODE_SEQUENCE(1);

    MODE_manager_bis;

    % generate random number between -180 and +180     
    att.alpha = 360*rand - 180;   % Yaw  
    att.beta = 360*rand - 180;     % Pitch   
    att.gamma = 360*rand - 180;    % Roll   


    %Initial angular velocities in each axis (x,y,z) of body frame [degrees/sec]
    att.wx0 = 40*rand - 20;    % generate random number between -5 and +5    
    att.wy0 = 40*rand - 20;
    att.wz0 = 40*rand - 20;


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
end

%Data processing
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

save('LQR_Sim3_changing_x3_modes_9orbits_n180.mat');
