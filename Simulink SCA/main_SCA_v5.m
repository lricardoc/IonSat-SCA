%% Initialization parameters
clear
close						

%Orbit: Initialisation of Keplerian parameters
orbit.a = 6678;     %semimajor axis [km]
orbit.e = 0.001;    %eccentricity
orbit.i = 98;     %inclination [degrees]
orbit.O = 10;      %Right ascension of the right ascending node [degrees] %max 197, min 300.5 %181
orbit.o = 90;      %Argument of the perigee [degrees]                      %max 90, min 0      %90
orbit.nu = 0;       %True anomaly [degrees]
%beta angle  ~9 deg. 21/3, i=98, O=10, 
%beta angle ~29 deg. 21/3, i=98, O=30,
%beta angle ~48 deg. 21/3, i=98, O=50,  
%i = 98: notice that for the date 21/3 (equinox) the RAAN (O) is close to the beta angle.

%Initialisation of date
%date.year = 2022;
date.year = 2024;
%date.month = 1;
date.month = 3;
%date.day = 1;
date.day = 21;
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
MODE = 1;   %"sun-aero" pointing mode
% 1: "orbital" Reference quaternion is aligned with ORF. 
% 2: "sun-aero" Reference quaternion is such that x is aligned with velocity 
%and z is aligned as best as possible with the sun direction to maximize the power generation 
% 3: "sun pointing" Reference quaternion is such that z is aligned with with 
%the sun direction to maximize the power generation and x is aligned as best 
%as possible with the velocity direction.
% 4: "aero-drag" Reference quaternion is similar than in Case 2, but rotated 
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
    att.q_i2r = (quatinv(q_s2o))';
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
    att.q_i2r = (quatinv(q_s2i))';
end

if MODE == 4
    x_sa = orbit.r/norm(orbit.r);
    z_sa = orbit.v/norm(orbit.v);
    y_sa = cross(z_sa,x_sa);
    DCM_sa=horzcat(x_sa,y_sa,z_sa);
    q_s2i = dcm2quat(DCM_sa);
    att.q_i2r = (quatinv(q_s2i))';
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
    att.q_i2r = (quatinv(q_s2i))';
end


%Attitude: Initialisation of angles and rotational speeds:
att.alpha = rand*180;         %Initial orientation Yaw [deg]  20
att.beta = rand*90;         %Initial orientation Pitch [deg] 80  sould not exceed +-90 deg.
att.gamma = 50*rand;         %Initial orientation Roll [deg] 50 
% if abs(att.beta)>90     %UNSURE if this is correct
%     att.beta=-att.beta;
%     att.alpha=att.alpha+180;
%     att.gamma=att.gamma+180;
% end
    
%Initial angular velocities in each axis (x,y,z) of body frame [degrees/sec]
att.wx0 = -0.1;        
att.wy0 = 0.1;
att.wz0 = 0.1;
%works: 10, -20, 30 and -1, 1, 1. mode 2, not 4
% %Initial angular velocities in each axis (x,y,z) of body frame [degrees/sec]
% att.wx0 = -4;        
% att.wy0 = 5;
% att.wz0 = 3;

%time
TimeStep = 1;        %fixed-step size in solver, Default time step=0.25
Torbit=2*pi*sqrt((orbit.a)^3/(3.986004418E5));
N_orbits = 2;           %number of orbits to be simulated
%Time spent performing the simulation in seconds (one orbit is ~5400 s):
t_sim = N_orbits*Torbit;

%% Load other parameters
load('SatConstant_Updated_04-2023.mat')

% Other blocks configuration

%%% GET PROPER IGRF COEFFICIENTS %%%
date_IGRF = [date.year,date.month,date.day];
%Determine the order of approximation for IGRF model of the earth's
%magnetic field nmax (must be equal or less than 13)
nmax = 2;
%Determine whether you are using full IGRF model or dipole approximation
%1 for full model, 0 for dipole approximation
Use_IGRF = 1; 


%RWA
% Calculate the pseudo inverse matrix
Pinv_RW_repartition     %executes the following code lines:
% alpha = 26.6 * pi/180;      %from deg to rad
% sat.wheel.repartition_matrix_4RW = [[cos(alpha),      0,    -cos(alpha),      0    ];...
%                                     [     0,     cos(alpha),     0,     -cos(alpha)];...
%                                     [sin(alpha), sin(alpha), sin(alpha), sin(alpha)]];
% sat.wheel.repartition_metrix_4RW_inverse = pinv(sat.wheel.repartition_matrix_4RW);

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
%% Simulation
Control_v7_PID                %MATLAB R2020b, latest one working fine



