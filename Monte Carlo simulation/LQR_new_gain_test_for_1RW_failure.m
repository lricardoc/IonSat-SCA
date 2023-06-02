

clear
close
%Initialize parameters				
%Load IONSat parameters
load('SatConstant_Updated_04-2023.mat')

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

%LQR controller feedback gain
%If the Inertia is fixed choose between the following two feedback gain matrix
%Keep in mind that there might be other good performing parameters

% K = [0.0013  0  0  0.0111  0  0;
%     0  0.0016  0  0  0.0129  0;
%     0  0  0.0017  0  0  0.0157]; %q=0.004 r=1000


%Retrieve feedback gain data from the twt file
Feedback_gain = readtable('LQR_feedback_gain_updated_parameter_q0.004_r1000-10000.txt');

kq1 =  Feedback_gain.Var3(1);
kq2 =  Feedback_gain.Var4(1);
kq3 =  Feedback_gain.Var5(1);
kw1 =  Feedback_gain.Var6(1);
kw2 =  Feedback_gain.Var7(1);
kw3 =  Feedback_gain.Var8(1);


K = [kq1  0   0  kw1  0   0;
      0  kq2  0   0  kw2  0;
      0   0  kq3  0   0  kw3]; %q=0.004 r=4000


%Monte Carlo simulation
n = 10; % number of simulation, it take between 95 and 120 minutes for n=100

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
RW_FAILURE_MATRIX = zeros(4,n);

for i = 1:n
    %Configure the RW that failed 
    RW_failed = randi([1, 4]);           %randomly generate the index of the failed RW
    RW_failure_matrix = ones(1,4);
    RW_failure_matrix(RW_failed) = 0;    %if 0, the RW failed 
    % Add noise to initial conditions
    %Randomize IONSat's inertia at -+20% of the fixed inertia
    % If the Inertia is fixed for all n simulations, comment the following 9 lines
    I_xx =  0.0702*8/10 + 0.0702*rand*2/5;
    I_yy =  0.113*8/10 + 0.113*rand*2/5;
    I_zz =  0.16*8/10 + 0.16*rand*2/5;
    I_xy =  0.0017*8/10 + 0.0017*rand*2/5;
    I_xz = -0.0023*8/10 + 0.0023*rand*2/5;
    I_yz = -0.0003*8/10 + 0.0003*rand*2/5;
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
    
%     Feedback_gain(1,i) = K(1);
%     Feedback_gain(2,i) = K(5);
%     Feedback_gain(3,i) = K(9);
%     Feedback_gain(4,i) = K(10);
%     Feedback_gain(5,i) = K(14);
%     Feedback_gain(6,i) = K(18);


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
 
    RW_FAILURE_MATRIX(:,i) = transpose(RW_failure_matrix);

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

save('LQR_Monte_Carlo_test_new_gain_1RW_failure_q0.004_r1000_n10.mat');