%% Clear workspace and figures
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

%% Orbit and attitude parameters
%Orbit: Initialisation of Keplerian parameters
orbit.a = 6678;     %semimajor axis [km]
orbit.e = 0.001;     %eccentricity
orbit.i = 51.6;     %inclination [degrees]
orbit.O = 146;      %Right ascension of the right ascending node [degrees] %max 197, min 300.5 %181
orbit.o = 344;      %Argument of the perigee [degrees]                      %max 90, min 0      %90
orbit.nu = 0;       %True anomaly [degrees]

%Initialisation of date
date.year = 2022;
date.month = 1;
date.day = 1;
date.hours = 0;
date.minutes = 0;
date.seconds = 0;




%Get initial position and velocity
[orbit.r,orbit.v] = orb2rv_init(orbit.a,orbit.e,orbit.i*pi/180,orbit.O*pi/180,orbit.o*pi/180,orbit.nu*pi/180,0,0,0);
%Get Sun initial direction
%Implement the position of the Sun with respect to the Earth for initialization date with DE405.
orbit.sun_ECI_0 = (planetEphemeris(juliandate(date.year,date.month,date.day),'Earth','Sun'))';
orbit.sun_ECI_0 = orbit.sun_ECI_0/norm (orbit.sun_ECI_0);   %get unit vector

%POINTING MODE
MODE = 2;   %"sun-aero" pointing mode
% 1: "orbital" Reference quaternion is aligned with ORF. 
% 2: "sun-aero" Reference quaternion is such that x is aligned with velocity 
%and z is aligned as best as possible with the sun direction to maximize the power generation 
% 3: "sun pointing" Reference quaternion is such that z is aligned with with 
%the sun direction to maximize the power generation and x is aligned as best 
%as possible with the velocity direction. 
% 4: "aero-drag" Reference quaternion is similar than in Case 1, but rotated 
%90° along the y axis, therefore, the reference quaternion is an attitude for 
%maximizing the drag surface. 
% 5: "retrogade firing" Reference quaternion is similar than in Case 1, but 
%rotated 180° along the z axis, therefore, the reference quaternion is an 
%attitude for retrograde propulsion. (??should be like case 2 but rotated?)
% 6: Reference quaternion is static [1 0 0 0], but this time is the only case 
%where the state of the B-dot is enabled (1). 
% 7: Reference quaternion is a custom quaternion that has to be defined by the 
%user as an input to the mission block. 
% 8: "sun-drag" Reference quaternion is such that z is aligned with 
%the velocity direction to maximize the drag and x is aligned as best 
%as possible with the velocity direction. 

if MODE == 1
    x_sa = orbit.v/norm(orbit.v);
    z_sa = orbit.r/norm(orbit.r);
    y_sa = -cross(x_sa,z_sa);
    DCM_orb = horzcat(x_sa,y_sa,z_sa);
    q_s2o = dcm2quat(DCM_orb);
    att.q_i2r = (quatinv (q_s2o))';
end

if MODE == 2
    x_sa = orbit.v/norm(orbit.v);
    z_sa = orbit.sun_ECI_0 - dot(x_sa,orbit.sun_ECI_0)*x_sa;
    z_sa = z_sa/norm(z_sa);
    y_sa = cross(z_sa,x_sa);
    DCM_sa = horzcat(x_sa,y_sa,z_sa);
    q_s2i = dcm2quat(DCM_sa);
    att.q_i2r = (quatinv(q_s2i))';
end

if MODE == 3
    z_sa = orbit.sun_ECI_0/norm(orbit.sun_ECI_0);
    x_sa = dot(orbit.v,z_sa)*z_sa - orbit.v;
    x_sa = x_sa/norm(x_sa);
    y_sa = cross(z_sa,x_sa);
    DCM_sa=horzcat(x_sa,y_sa,z_sa);
    q_s2i = dcm2quat(DCM_sa);
    att.q_i2r = (quatinv (q_s2i))';
end

if MODE == 4
    x_sa = orbit.r/norm(orbit.r);
    z_sa = orbit.v/norm(orbit.v);
    y_sa = cross(z_sa,x_sa);
    DCM_sa=horzcat(x_sa,y_sa,z_sa);
    q_s2i = dcm2quat(DCM_sa);
    att.q_i2r = (quatinv (q_s2i))';
end

if MODE == 5
    x_sa = - orbit.v/norm(orbit.v);
    z_sa = orbit.sun_ECI_0 - dot(x_sa,orbit.sun_ECI_0)*x_sa;
    z_sa = z_sa/norm(z_sa);
    y_sa = cross(z_sa,x_sa);
    DCM_sa = horzcat(x_sa,y_sa,z_sa);
    q_s2i = dcm2quat(DCM_sa);
    att.q_i2r = (quatinv(q_s2i))';
end

if MODE == 6
    att.q_i2r = [1 0 0 0];
end

if MODE == 7
    prompt = "What is the desired quaternion value? ";
    att.q_i2r = input(prompt);
end

if MODE == 8
    z_sa = orbit.sun_ECI_0/norm(orbit.sun_ECI_0);
    nadir = - orbit.r/norm(orbit.r);
    x_sa = dot(nadir,z_sa)*z_sa - nadir;
    x_sa = x_sa/norm(x_sa);
    y_sa = cross(z_sa,x_sa);
    DCM_sa=horzcat(x_sa,y_sa,z_sa);
    q_s2i = dcm2quat(DCM_sa);
    att.q_i2r = (quatinv (q_s2i))';
end


%Attitude: Initialisation of angles and rotational speeds:
%Initial orientation in ZYX (alpha,beta,gamma) Euler angles [degrees]
att.alpha = 40;     
att.beta = -10;
att.gamma = 60;

%Initial angular velocities in each axis (x,y,z) of body frame [degrees/sec]
att.wx0 = -0.01;        
att.wy0 = 0.01;
att.wz0 = 0.01;

%% Declare variables
%time step based on gyro sampling frequency: 
%TimeStep = 0.25;        %fixed-step size in solver, Default time step=0.25
TimeStep = 0.5;        %fixed-step size in solver, Default time step=0.25
Torbit=2*pi*sqrt((orbit.a)^3/(3.986004418E5));
N_orbits = 1;           %number of orbits to be simulated
%Time spent performing the simulation in seconds (one orbit is ~5400 s):
t_sim = N_orbits*Torbit;
%t_sim = 10800;
%OrbitSize = 5400;

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
Enable_control = 0;
%Enable whether the sensors are considering eclipses or not
Enable_eclipse = 1;




%Extended Kalman Filter
global dt Tss Ts q_hat0 b_hat0 W6x6 Vmtm Vfss Vcss  
%global PSDgyro PSDmtm PSDfss PSDcss PSDbias
%Sensor parameters
%Sampling times:
Tss=0.5;         %sampling time for MTM, FSS, CSS could be 1 not 0.5
Ts=0.5;          %sampling time for Gyro
%Sensor Noises
%a) Magnetometer (MTM)
sat.sensors.mag_sigma=5.0e-08;
PSDmtm=sat.sensors.mag_sigma^2*Tss;
%b) Gyrometer 
sat.sensors.gyro_sigma=2.620e-04; %original value
sat.sensors.gyro_sigma=0.2e-04; %to test
sat.sensors.gyro_bias=1.160e-04; %original value
sat.sensors.gyro_bias=0.160e-05; %to test
PSDgyro=sat.sensors.gyro_sigma^2*Ts;
PSDbia4s=sat.sensors.gyro_bias^2*Ts;
b_offset=2e-4;
b_offset=2e-2;
b_offset=sat.sensors.gyro_bias;
%b_offset=0e-2;
b_offset2=-1e-2;    %for the triangular signal
b_offset2=0;    %for the triangular signal
%b_offset2=sat.sensors.gyro_bias;
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
sigma_w = 1e-4;4
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

%% Controller
%LDR controller feedback gain
K = [0.00053417  0  0  0.0070506  0  0;
    0  0.00071733 0  0  0.00840481  0;
    0  0  0.00081350 0  0  0.010476]; %q=0.004 r=4000

%choose the controller to use : if PID then controller=1, 
%if LQR then controller=0
controller = 1;

%choose the kalman filter : if UKF then  Kalman_filter = 1,
%if MEKF then  Kalman_filter = 0,
Kalman_filter = 0;

% %% Open the simulink model
% IonSatSimulationF
% 
% %% Load the simulink model
% load_system("IonSatSimulationF.slx")
% 
% %% Run the simulink 
% simOut = sim("IonSatSimulationF");