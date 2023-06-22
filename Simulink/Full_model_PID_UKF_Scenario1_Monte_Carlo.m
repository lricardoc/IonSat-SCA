%% PID UKF Scenario1 : Robustness to the inertia varation

clear all
close all
clc

%% Loading the Data
%Loads the necessary physical variables describing the satellite and its
%components
load('SatConstant_Updated_04-2023.mat')
load('data.mat')
load('Simulation.mat')
load('C.mat')
load('F.mat')
load('P.mat')
load('T.mat')
load('workspace.mat')
%Not necessary while perturbations not implemented
load('LOAS.mat')


%% Declare variables
%time step based on gyro sampling frequency: 
TimeStep = 1;        %fixed-step size in solver, Default time step=0.25
Torbit=2*pi*sqrt((orbit.a)^3/(3.986004418E5));
N_orbits = 1;           %number of orbits to be simulated
t_sim = N_orbits*Torbit;


%% Other blocks configuration
%Determine the order of approximation for IGRF model of the earth's
%magnetic field nmax (must be equal or less than 13)
nmax = 2;
%Determine whether you are using full IGRF model or dipole approximation
%1 for true, 0 for false
Use_IGRF = 1; 
%Determine whether you are using full IGRF model or dipole approximation
%for the Kalman Filter; 1 for true, 0 for false
Use_IGRF_KF = 1; 
%Decide whether you'd like to use control or not
%A value of 1 (True) entails that you'd like to simulate a realistic model
%for the control system, and 0 (False) means you assume ideal no control conditions
% Enable_control = 1;
%Enable whether the sensors are considering eclipses or not
Enable_eclipse = 1;



%Extended Kalman Filter
global dt Tss Ts q_hat0 b_hat0 W6x6 Vmtm Vfss Vcss  
%global PSDgyro PSDmtm PSDfss PSDcss PSDbias
%Sensor parameters
%Sampling times:
Tss=1;         %sampling time for MTM, FSS, CSS could be 1 not 0.5
Ts=1;          %sampling time for Gyro
%Sensor Noises
%a) Magnetometer (MTM)
sat.sensors.mag_sigma=5.0e-08;
PSDmtm=sat.sensors.mag_sigma^2*Tss;
%b) Gyrometer 
% sat.sensors.gyro_sigma=2.620e-04; %original value
% sat.sensors.gyro_sigma=0.2e-04; %to test
% sat.sensors.gyro_bias=1.160e-04; %original value
% sat.sensors.gyro_bias=0.160e-05; %to test
% PSDgyro=sat.sensors.gyro_sigma^2*Ts;
% PSDbias=sat.sensors.gyro_bias^2*Ts;
% b_offset=2e-4;
% b_offset=2e-2;
% b_offset=sat.sensors.gyro_bias;
% %b_offset=0e-2;
% b_offset2=-1e-2;    %for the triangular signal
% b_offset2=0;    %for the triangular signal
% %b_offset2=sat.sensors.gyro_bias;

%c) Fin Sun Sensor (FSS)
sat.sensors.sun_sigma=0.001164;
PSDfss=sat.sensors.sun_sigma^2*Tss;
%d) Coarse Sun Sensor (CSS)
sat.sensors.sun_coarse_sigma=0.1745;
PSDcss=sat.sensors.sun_coarse_sigma^2*Tss;


%Other Kalman Filter Parameters:
%initial values
dt=Ts;
q_hat0=[1;0;0;0]; 
%q_hat0=[0.6;0.8;0.13;0.05]; q_hat0=q_hat0/norm(q_hat0); 
q_hat0=[0.56 0.81 0.10 0.09]; q_hat0=q_hat0/norm(q_hat0); %ok
% q_hat0=[0.56 0.81 0.05 0.09]; q_hat0=q_hat0/norm(q_hat0); %ok ok
% q_hat0=[0.92 0.32 0.05 0.14]; q_hat0=q_hat0/norm(q_hat0); %very close 0
b_hat0=[0;0;0];
%b_hat0=[0.2e-3;0.2e-3;0.2e-3];


%System state noise matrix, the covariance of the process noise matrix Wd
Vg=sat.sensors.gyro_sigma^2;  %ok
Vg=(5e-5)^2;
Vb=sat.sensors.gyro_bias^2;   %ok
Vb=(5.16e-6)^2; %OK
Vb=(0.16e-5)^2; %OK
%Vb=1e-12;

Q11 = ((Vg*dt+(1/3)*Vb*dt^3))*eye(3);
Q12 = (-1/2*Vb*dt^2)*eye(3);
Q22 = (Vb*dt)*eye(3);
Q = [ Q11 , Q12 ; Q12 , Q22];

G=diag([-1 -1 -1 1 1 1]);
W6x6 = G*Q*G';

%for tests
%A) FSS
% Vfss=sat.sensors.sun_sigma^2;
% Vfss=0.005^2; %repeat again here modify %ok 0.0012
% Vfss=0.04^2; %repeat again here modify %ok 0.0012
% Vfss=0.002^2; %repeat again here modify %ok 0.0012
% V=Vfss*eye(3);

%B) MTM and FSS
% Vmtm=sat.sensors.mag_sigma^2; 
% Vmtm=(2e-7)^2;
% Vfss=sat.sensors.sun_sigma^2;
% Vfss=0.002^2; %repeat again here modify %ok 0.0012
% V=[Vmtm*eye(3),zeros(3);zeros(3),Vfss*eye(3)]; 

%C) MTM, CSS and FSS
Vmtm=sat.sensors.mag_sigma^2; 
%Vmtm=(2e-7)^2;
Vfss=sat.sensors.sun_sigma^2;
%Vfss=0.002^2; %repeat again here modify %ok 0.0012
Vcss=sat.sensors.sun_coarse_sigma^2;
Vcss=0.01745^2;
%Vcss=(1e-1)^2;





%Define initial values for Kalman filter parameters
x0 = [0.207740030841190 0.530411062089572 -0.675665322016929 0.467957858597196 1e-3 1e-3 1e-3];
P0 = diag([1e-5 1e-5 1e-5 1e-5 1e-5 1e-5]);
What0 = [-8.56493764825016e-05 -0.000707789025389356 -0.000815029912348624];
%Identity matrix with same number of dimensions as P
CovId = eye(6);

dt = TimeStep;
sigma_w = 1e-4;
sigma_b = 1e-5;
sigma_s = 1e-3;
sigma_m = 1e-6;
sigma_sc = 1e-1;

Gk = diag([-1 -1 -1 1 1 1]);
Q1 = ((sigma_w^2)*dt + (sigma_b^2)*(dt^3)/3) .* eye(3);
Q2 = -((sigma_b^2)*(dt^2)/2) .* eye(3);
Q3 = Q2;
Q4 = ((sigma_b^2)*dt) .* eye(3);
Qk = [Q1 Q2; Q3 Q4];
GQG = Gk*Qk*(Gk.');

%RWA Calculate the pseudo inverse matrix
alpha = 26.6 * pi/180;      %from deg to rad
sat.wheel.repartition_matrix_4RW = [[cos(alpha),      0,    -cos(alpha),      0    ];...
                                    [     0,     cos(alpha),     0,     -cos(alpha)];...
                                    [sin(alpha), sin(alpha), sin(alpha), sin(alpha)]];
sat.wheel.repartition_metrix_4RW_inverse = pinv(sat.wheel.repartition_matrix_4RW);

%% Other disturbance block parameters
%%% Model of Perturbation Torques
%Magnetic Torque Residual dipole
sat.residual_dipole=[0.08;0.08;0.08]; %residual magnetic dipole in [A*m^2]
%Aerodynamic Torque Constants
%load('LOAS.mat')`
load('IonSat_6U.mat');
IonSataero.T = SNAP_aeromodel.T;
IonSataero.El = SNAP_aeromodel.Az;
IonSataero.Az = SNAP_aeromodel.El;
IonSataero.Tx = SNAP_aeromodel.T_x;
IonSataero.Ty = SNAP_aeromodel.T_y;
IonSataero.Tz = SNAP_aeromodel.T_z;
IonSataero.av_density_vs_alt = SNAP_aeromodel.av_density_vs_alt;
IonSataero.alt_range = SNAP_aeromodel.alt_range;

%% Thruster activation
thruster_on_off = 0;

%Thruster direction and activation:
sat.thruster.force=0.00075; % Force of thruster in [N]
d=1*pi/180;           %deviation in rads of the thruster wrt to x axis in plane XY
alpha=80*pi/180;       %rotation of deviation around x axis
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





%% Monte Carlo simulation
n = 100; % number of simulation, take around 12 minutes for n=10

%initialize lists of data
n_points = round(t_sim) + 1; % number of points to save per simulations

GYRO_sigma = zeros(1,n);
Gyro_bias = zeros(1,n);

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


Q_real_0 = zeros(n_points,n);
Q_real_1 = zeros(n_points,n);
Q_real_2 = zeros(n_points,n);
Q_real_3 = zeros(n_points,n);

Q_estimated_0 = zeros(n_points,n);
Q_estimated_1 = zeros(n_points,n);
Q_estimated_2 = zeros(n_points,n);
Q_estimated_3 = zeros(n_points,n);


Q_reference_0 = zeros(n_points,n);
Q_reference_1 = zeros(n_points,n);
Q_reference_2 = zeros(n_points,n);
Q_reference_3 = zeros(n_points,n);


Attitude_real_x = zeros(n_points,n);
Attitude_real_y = zeros(n_points,n);
Attitude_real_z = zeros(n_points,n);

Attitude_estimated_x = zeros(n_points,n);
Attitude_estimated_y = zeros(n_points,n);
Attitude_estimated_z = zeros(n_points,n);

Attitude_reference_x = zeros(n_points,n);
Attitude_reference_y = zeros(n_points,n);
Attitude_reference_z = zeros(n_points,n);

Attitude_real_vs_estimated_x = zeros(n_points,n);
Attitude_real_vs_estimated_y = zeros(n_points,n);
Attitude_real_vs_estimated_z = zeros(n_points,n);

Attitude_real_vs_reference_x = zeros(n_points,n);
Attitude_real_vs_reference_y = zeros(n_points,n);
Attitude_real_vs_reference_z = zeros(n_points,n);

Attitude_estimated_vs_reference_x = zeros(n_points,n);
Attitude_estimated_vs_reference_y = zeros(n_points,n);
Attitude_estimated_vs_reference_z = zeros(n_points,n);



Angular_velocity_real_x = zeros(n_points,n);
Angular_velocity_real_y = zeros(n_points,n);
Angular_velocity_real_z = zeros(n_points,n);

Angular_velocity_estimated_x = zeros(n_points,n);
Angular_velocity_estimated_y = zeros(n_points,n);
Angular_velocity_estimated_z = zeros(n_points,n);

Angular_velocity_reference_x = zeros(n_points,n);
Angular_velocity_reference_y = zeros(n_points,n);
Angular_velocity_reference_z = zeros(n_points,n);

Angular_velocity_real_vs_estimated_x = zeros(n_points,n);
Angular_velocity_real_vs_estimated_y = zeros(n_points,n);
Angular_velocity_real_vs_estimated_z = zeros(n_points,n);

Angular_velocity_real_vs_reference_x = zeros(n_points,n);
Angular_velocity_real_vs_reference_y = zeros(n_points,n);
Angular_velocity_real_vs_reference_z = zeros(n_points,n);

Angular_velocity_estimated_vs_reference_x = zeros(n_points,n);
Angular_velocity_estimated_vs_reference_y = zeros(n_points,n);
Angular_velocity_estimated_vs_reference_z = zeros(n_points,n);


% Orbit_altitude = zeros(1,n);
% 
Inertia =  zeros(6,n);

I_xx =  0.0702;
I_yy =  0.113;
I_zz =  0.16;
I_xy =  0.0017;
I_xz = -0.0023;
I_yz = -0.0003;
sat.inertia = [I_xx I_xy I_xz;
               I_xy I_yy I_yz;
               I_xz I_yz I_zz];
for i = 1:n
    % Add noise to initial conditions

    sat.sensors.gyro_sigma = (2.620e-04)*0.5 +rand*(2.620e-04); %to test
    sat.sensors.gyro_bias=(1.160e-04)*0.5 +rand*(1.160e-04); %to test   
    
    PSDgyro=sat.sensors.gyro_sigma^2*Ts;
    PSDbias=sat.sensors.gyro_bias^2*Ts;
    
    b_offset=sat.sensors.gyro_bias;
    b_offset2=-1e-2;    %for the triangular signal
    
    
    GYRO_sigma(i) = sat.sensors.gyro_sigma;
    Gyro_bias(i) = sat.sensors.gyro_bias;
    
    %Randomize IONSat's inertia at -+20% of the fixed inertia
    I_xx =  0.0702*8/10 + 0.0702*rand*2/5;
    I_yy =  0.113*8/10 + 0.113*rand*2/5;
    I_zz =  0.16*8/10 + 0.16*rand*2/5;
    I_xy =  0.0017*8/10 + 0.0017*rand*2/5;
    I_xz = -0.0023*8/10 + 0.0023*rand*2/5;
    I_yz = -0.0003*8/10 + 0.0003*rand*2/5;
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
    
    MODE = 4;
    MODE_manager;
    
    %intial angles
    att.alpha = 1;   % Yaw  
    att.beta = -1;     % Pitch   
    att.gamma = 1;    % Roll   

    %Initial angular velocities in each axis (x,y,z) of body frame [degrees/sec]
    att.wx0 = 0.1;    
    att.wy0 = -0.1;
    att.wz0 = 0.1;

    %Save the inital conditions
    Alpha0(i) = att.alpha;
    Beta0(i) = att.beta;
    Gamma0(i) = att.gamma;
    WX0(i) = att.wx0;
    WY0(i) = att.wy0;
    WZ0(i) = att.wz0;
    N(i)=i;
% Orbit_altitude(i) = orbit.a - 6378;
    
    Inertia(1,i) = I_xx;
    Inertia(2,i) = I_yy;
    Inertia(3,i) = I_zz;
    Inertia(4,i) = I_xy;
    Inertia(5,i) = I_xz;
    Inertia(6,i) = I_yz;


    % run the simulink 
    simResults = sim('IonSatSimulationF_UKF.slx');
    
    
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
    
   
    Q_real_0(:,i) = simResults.q_real.Data(:,1);
    Q_real_1(:,i) = simResults.q_real.Data(:,2);
    Q_real_2(:,i) = simResults.q_real.Data(:,3);
    Q_real_3(:,i) = simResults.q_real.Data(:,4);

    Q_estimated_0(:,i) = simResults.q_estimated.Data(:,1);
    Q_estimated_1(:,i) = simResults.q_estimated.Data(:,2);
    Q_estimated_2(:,i) = simResults.q_estimated.Data(:,3);
    Q_estimated_3(:,i) = simResults.q_estimated.Data(:,4);


    Q_reference_0(:,i) = simResults.q_ref.Data(1,:);
    Q_reference_1(:,i) = simResults.q_ref.Data(2,:);
    Q_reference_2(:,i) = simResults.q_ref.Data(3,:);
    Q_reference_3(:,i) = simResults.q_ref.Data(4,:);   
    
    Angular_velocity_real_x(:,i) = rad2deg(simResults.w_real.Data(1,:));
    Angular_velocity_real_y(:,i) = rad2deg(simResults.w_real.Data(2,:));
    Angular_velocity_real_z(:,i) = rad2deg(simResults.w_real.Data(3,:));

    Angular_velocity_estimated_x(:,i) = rad2deg(simResults.w_estimated.Data(:,1));
    Angular_velocity_estimated_y(:,i) = rad2deg(simResults.w_estimated.Data(:,2));
    Angular_velocity_estimated_z(:,i) = rad2deg(simResults.w_estimated.Data(:,3));

    Angular_velocity_reference_x(:,i) = rad2deg(simResults.w_ref.Data(:,1));
    Angular_velocity_reference_y(:,i) = rad2deg(simResults.w_ref.Data(:,2));
    Angular_velocity_reference_z(:,i) = rad2deg(simResults.w_ref.Data(:,3));
    
    Angular_velocity_real_vs_estimated_x(:,i) = Angular_velocity_real_x(:,i) - Angular_velocity_estimated_x(:,i);
    Angular_velocity_real_vs_estimated_y(:,i) = Angular_velocity_real_y(:,i) - Angular_velocity_estimated_y(:,i);
    Angular_velocity_real_vs_estimated_z(:,i) = Angular_velocity_real_z(:,i) - Angular_velocity_estimated_z(:,i);

    Angular_velocity_real_vs_reference_x(:,i) = Angular_velocity_real_x(:,i) - Angular_velocity_reference_x(:,i);
    Angular_velocity_real_vs_reference_y(:,i) = Angular_velocity_real_y(:,i) - Angular_velocity_reference_y(:,i);
    Angular_velocity_real_vs_reference_z(:,i) = Angular_velocity_real_z(:,i) - Angular_velocity_reference_z(:,i);

    Angular_velocity_estimated_vs_reference_x(:,i) = Angular_velocity_estimated_x(:,i) - Angular_velocity_reference_x(:,i);
    Angular_velocity_estimated_vs_reference_y(:,i) = Angular_velocity_estimated_y(:,i) - Angular_velocity_reference_y(:,i);
    Angular_velocity_estimated_vs_reference_z(:,i) = Angular_velocity_estimated_z(:,i) - Angular_velocity_reference_z(:,i);
         
end

%% Data processing
PID_average_magnitude_angle_error = zeros(n,3);

PID_average_magnitude_angular_velocities_error = zeros(n,3);

PID_RW_average_power_consumtion = zeros(1,n);

Attitude_average_mangnitude_real_vs_estimated = zeros(n,3);
Attitude_average_mangnitude_real_vs_reference = zeros(n,3);
Attitude_average_mangnitude_estimated_vs_reference = zeros(n,3);

Angular_velocity_average_mangnitude_real_vs_estimated = zeros(n,3);
Angular_velocity_average_mangnitude_real_vs_reference = zeros(n,3);
Angular_velocity_average_mangnitude_estimated_vs_reference = zeros(n,3);



    

for k =1:n
    for j=1:n_points 
    Euler_real = Quat2Euler([Q_real_0(j,k) Q_real_1(j,k) Q_real_2(j,k) Q_real_3(j,k)]);
    Attitude_real_x(j,k) = Euler_real(1);
    Attitude_real_y(j,k) = Euler_real(2);
    Attitude_real_z(j,k) = Euler_real(3);
    
    Euler_estimated = Quat2Euler([Q_estimated_0(j,k) Q_estimated_1(j,k) Q_estimated_2(j,k) Q_estimated_3(j,k)]);
    Attitude_estimated_x(j,k) = Euler_estimated(1);
    Attitude_estimated_y(j,k) = Euler_estimated(2);
    Attitude_estimated_z(j,k) = Euler_estimated(3);
       
    Euler_reference = Quat2Euler([Q_reference_0(j,k) Q_reference_1(j,k) Q_reference_2(j,k) Q_reference_3(j,k)]);
    Attitude_reference_x(j,k) = Euler_reference(1);
    Attitude_reference_y(j,k) = Euler_reference(2);
    Attitude_reference_z(j,k) = Euler_reference(3);
    end
    
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
    
    
    Attitude_real_vs_estimated_x(:,k) = Attitude_real_x(:,k) - Attitude_estimated_x(:,k);
    Attitude_real_vs_estimated_y(:,k) = Attitude_real_y(:,k) - Attitude_estimated_y(:,k);
    Attitude_real_vs_estimated_z(:,k) = Attitude_real_z(:,k) - Attitude_estimated_z(:,k);

    Attitude_real_vs_reference_x(:,k) = Attitude_real_x(:,k) - Attitude_reference_x(:,k);
    Attitude_real_vs_reference_y(:,k) = Attitude_real_y(:,k) - Attitude_reference_y(:,k);
    Attitude_real_vs_reference_z(:,k) = Attitude_real_z(:,k) - Attitude_reference_z(:,k);

    Attitude_estimated_vs_reference_x(:,k) = Attitude_estimated_x(:,k) - Attitude_reference_x(:,k);
    Attitude_estimated_vs_reference_y(:,k) = Attitude_estimated_y(:,k) - Attitude_reference_y(:,k);
    Attitude_estimated_vs_reference_z(:,k) = Attitude_estimated_z(:,k) - Attitude_reference_z(:,k);
 
    %Compute the average magnitude (RMS) real attitude vs estimated
    Attitude_average_mangnitude_real_vs_estimated(k,1) = sqrt(mean(Attitude_real_vs_estimated_x(:,k).^2 ));
    Attitude_average_mangnitude_real_vs_estimated(k,2) = sqrt(mean(Attitude_real_vs_estimated_y(:,k).^2 ));
    Attitude_average_mangnitude_real_vs_estimated(k,3) = sqrt(mean(Attitude_real_vs_estimated_z(:,k).^2 ));
    
    %Compute the average magnitude (RMS) real attitude vs reference
    Attitude_average_mangnitude_real_vs_reference(k,1) = sqrt(mean(Attitude_real_vs_reference_x(:,k).^2 ));
    Attitude_average_mangnitude_real_vs_reference(k,2) = sqrt(mean(Attitude_real_vs_reference_y(:,k).^2 ));
    Attitude_average_mangnitude_real_vs_reference(k,3) = sqrt(mean(Attitude_real_vs_reference_z(:,k).^2 ));
   
    %Compute the average magnitude (RMS) estimated attitude vs reference
    Attitude_average_mangnitude_estimated_vs_reference(k,1) = sqrt(mean(Attitude_estimated_vs_reference_x(:,k).^2 ));
    Attitude_average_mangnitude_estimated_vs_reference(k,2) = sqrt(mean(Attitude_estimated_vs_reference_y(:,k).^2 ));
    Attitude_average_mangnitude_estimated_vs_reference(k,3) = sqrt(mean(Attitude_estimated_vs_reference_z(:,k).^2 ));    

    
    %Compute the average magnitude (RMS) real Angular_velocity vs estimated
    Angular_velocity_average_mangnitude_real_vs_estimated(k,1) = sqrt(mean(Angular_velocity_real_vs_estimated_x(:,k).^2 ));
    Angular_velocity_average_mangnitude_real_vs_estimated(k,2) = sqrt(mean(Angular_velocity_real_vs_estimated_y(:,k).^2 ));
    Angular_velocity_average_mangnitude_real_vs_estimated(k,3) = sqrt(mean(Angular_velocity_real_vs_estimated_z(:,k).^2 ));
    
    %Compute the average magnitude (RMS) real Angular_velocity vs reference
    Angular_velocity_average_mangnitude_real_vs_reference(k,1) = sqrt(mean(Angular_velocity_real_vs_reference_x(:,k).^2 ));
    Angular_velocity_average_mangnitude_real_vs_reference(k,2) = sqrt(mean(Angular_velocity_real_vs_reference_y(:,k).^2 ));
    Angular_velocity_average_mangnitude_real_vs_reference(k,3) = sqrt(mean(Angular_velocity_real_vs_reference_z(:,k).^2 ));
   
    %Compute the average magnitude (RMS) estimated Angular_velocity vs reference
    Angular_velocity_average_mangnitude_estimated_vs_reference(k,1) = sqrt(mean(Angular_velocity_estimated_vs_reference_x(:,k).^2 ));
    Angular_velocity_average_mangnitude_estimated_vs_reference(k,2) = sqrt(mean(Angular_velocity_estimated_vs_reference_y(:,k).^2 ));
    Angular_velocity_average_mangnitude_estimated_vs_reference(k,3) = sqrt(mean(Angular_velocity_estimated_vs_reference_z(:,k).^2 ));     
end

%% 
n_point = length(Attitude_real_vs_estimated_x(:,1));
Attitude_real_vs_estimated_process_x = zeros(n_point,n);
Attitude_real_vs_estimated_process_y = zeros(n_point,n);
Attitude_real_vs_estimated_process_z = zeros(n_point,n);

Attitude_average_mangnitude_real_vs_estimated_process = zeros(n,3);

for i=1:100
    parfor j=1:n_point
        if Attitude_real_vs_estimated_x(j,i)>180 
            Attitude_real_vs_estimated_process_x(j,i)=mod(360,Attitude_real_vs_estimated_x(j,i));
        elseif Attitude_real_vs_estimated_x(j,i)<-180
            Attitude_real_vs_estimated_process_x(j,i)=mod(-360,Attitude_real_vs_estimated_x(j,i));
        else
            Attitude_real_vs_estimated_process_x(j,i)=Attitude_real_vs_estimated_x(j,i);
        end    
        if Attitude_real_vs_estimated_y(j,i)>180 
            Attitude_real_vs_estimated_process_y(j,i)=mod(360,Attitude_real_vs_estimated_y(j,i));
        elseif Attitude_real_vs_estimated_y(j,i)<-180
            Attitude_real_vs_estimated_process_y(j,i)=mod(-360,Attitude_real_vs_estimated_y(j,i));
        else
            Attitude_real_vs_estimated_process_y(j,i)=Attitude_real_vs_estimated_y(j,i);
        end    
        if Attitude_real_vs_estimated_z(j,i)>180 
            Attitude_real_vs_estimated_process_z(j,i)=mod(360,Attitude_real_vs_estimated_z(j,i));
        elseif Attitude_real_vs_estimated_z(j,i)<-180
            Attitude_real_vs_estimated_process_z(j,i)=mod(-360,Attitude_real_vs_estimated_z(j,i));
        else
            Attitude_real_vs_estimated_process_z(j,i)=Attitude_real_vs_estimated_z(j,i);
        end
    end
    Attitude_average_mangnitude_real_vs_estimated_process(i,1) = sqrt(mean(Attitude_real_vs_estimated_process_x(:,i).^2 ));
    Attitude_average_mangnitude_real_vs_estimated_process(i,2) = sqrt(mean(Attitude_real_vs_estimated_process_y(:,i).^2 ));
    Attitude_average_mangnitude_real_vs_estimated_process(i,3) = sqrt(mean(Attitude_real_vs_estimated_process_z(:,i).^2 ));
    
end

save('PID_UKF_scenario1_inertia_pm20_SS_partially_active_n100.mat');

