%% TEST PID gain to cope with 1RW failure

clear
close
%Initialize parameters				
%Load IONSat parameters
load('SatConstant_Updated_04-2023.mat')

% PID gain
Kp_1RW = 0.2353;
Ki_1RW = 0.001961;
Kd_1RW = 7.059;


%Initial sat inertia
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
n = 100; % number of simulation, take around 12 minutes for n=10

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

Inertia =  zeros(6,n);

RW_FAILURE_MATRIX = zeros(4,n);

for i = 1:n
    %Configure the RW that failed 
    RW_failed = randi([1, 4]);           %randomly generate the index of the failed RW
    RW_failure_matrix = ones(1,4);
    RW_failure_matrix(RW_failed) = 0;    %if 0, the RW failed 
    % Add noise to initial conditions
    %Randomize IONSat's inertia at -+20% of the fixed inertia
    I_xx =  0.0702*9/10 + 0.0702*rand/5;
    I_yy =  0.113*9/10 + 0.113*rand/5;
    I_zz =  0.16*9/10 + 0.16*rand/5;
    I_xy =  0.0017*9/10 + 0.0017*rand/5;
    I_xz = -0.0023*9/10 + 0.0023*rand/5;
    I_yz = -0.0003*9/10 + 0.0003*rand/5;
    sat_inertia = [I_xx I_xy I_xz;
                   I_xy I_yy I_yz;
                   I_xz I_yz I_zz];


     %Orbit: Initialisation of Keplerian parameters
    orbit.a = 6378+300;
    orbit.e = 0.001;    %eccentricity
    orbit.i = 98;     %inclination [degrees]
    orbit.O = 10;      %Right ascension of the right ascending node [degrees] %max 197, min 300.5 %181
    orbit.o = 90;      %Argument of the perigee [degrees]                      %max 90, min 0      %90
    orbit.nu = 0;       %True anomaly [degrees]
    
    MODE_manager;
    
    %intial angles
    att.alpha = 5;   % Yaw  
    att.beta = -5;     % Pitch   
    att.gamma = 5;    % Roll   

    %Initial angular velocities in each axis (x,y,z) of body frame [degrees/sec]
    att.wx0 = 3;    
    att.wy0 = -3;
    att.wz0 = 3;

    %Save the inital conditions
    Alpha0(i) = att.alpha;
    Beta0(i) = att.beta;
    Gamma0(i) = att.gamma;
    WX0(i) = att.wx0;
    WY0(i) = att.wy0;
    WZ0(i) = att.wz0;
    N(i)=i;
    
    Inertia(1,i) = I_xx;
    Inertia(2,i) = I_yy;
    Inertia(3,i) = I_zz;
    Inertia(4,i) = I_xy;
    Inertia(5,i) = I_xz;
    Inertia(6,i) = I_yz;


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
    
    RW_FAILURE_MATRIX(:,i) = transpose(RW_failure_matrix);
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

save('PID_Monte_Carlo_test_new_gain_1RW_failure_Ku0.4_Pu240_n100.mat');
