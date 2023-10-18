 %MAIN CODE TO BE EXECUTED
clear all
close all

%Orbit simulation
%The object is assumed submitted only to gravity and it's mass is m in kg
m = 12; %mass (kg)
%Initialisation of Keplerian parameters
orbit.a = 6678;     %semimajor axis [km]
orbit.e = 0.001;    %eccentricity
orbit.i = 98;     %inclination [degrees]
orbit.O =  10;      %Right ascension of the right ascending node [degrees] %max 197, min 300.5 %181
orbit.o = 90;      %Argument of the perigee [degrees]                      %max 90, min 0      %90
orbit.nu = 0;       %True anomaly [degrees]
%beta angle  ~9 deg. 21/3, i=98, O=10, 
%beta angle ~29 deg. 21/3, i=98, O=30,
%beta angle ~48 deg. 21/3, i=98, O=50,  
%i = 98: notice that for the date 21/3 (equinox) the RAAN (O) is close to the beta angle.

%POINTING MODE
MODE = 2;   %"sun-aero" pointing mode


%Initialisation of date
date.year = 2024;
date.month = 3;
date.day = 21;
date.hours = 0;
date.minutes = 0;
date.seconds = 0;

%Simulation time parameters
N_orbits = 1;
%Torbit=90*60;       %1 orbit approx. 90 minutes
Torbit=2*pi*sqrt(orbit.a^3/(3.986004418E5));
%tsimulation=60*45;   %in [s] 1 orbit
t_sim=N_orbits*Torbit;
%tsimulation=10000;
%tsimulation=2700;
TimeStep = 1; %simulation time step (seconds)

%%% MAGNETIC FIELD MODELS - GET PROPER IGRF COEFFICIENTS %%%
date_IGRF = [date.year,date.month,date.day];
%"REAL" Magnetic Field
%Determine whether you are using full IGRF model or dipole approximation
%1 for full model, 0 for dipole approximation
Use_IGRF = 0; 

nmax = 2;

%"EMBEDDED" Magnetic Field
%Determine whether you are using full IGRF model or dipole approximation
%1 for true, 0 for false
Use_IGRF_KF = 0; 
%Determine the order of approximation for embedded IGRF model
nmax_KF = 2;


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
%Vg=sat.sensors.gyro_sigma^2;  %ok
Vg=(5e-5)^2;
%Vb=sat.sensors.gyro_bias^2;   %ok
%Vb=(5.16e-6)^2; %OK
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



%% Monte Carlo simulation
n = 100; % number of simulation, take around 12 minutes for n=10

%initialize lists of data
n_points = round(t_sim) + 1; % number of points to save per simulations

GYRO_sigma = zeros(1,n);
Gyro_bias = zeros(1,n);

N = zeros(1,n);

UKF_angle_error_rate_x = zeros(n_points,n);
UKF_angle_error_rate_y = zeros(n_points,n);
UKF_angle_error_rate_z = zeros(n_points,n);

UKF_angular_velocity_error_rate_x = zeros(n_points,n);
UKF_angular_velocity_error_rate_y = zeros(n_points,n);
UKF_angular_velocity_error_rate_z = zeros(n_points,n);

% UKF_angle_estimation_error_x = zeros(n_points,n);
% UKF_angle_estimation_error_y = zeros(n_points,n);
% UKF_angle_estimation_error_z = zeros(n_points,n);

UKF_bias_x = zeros(n_points,n);
UKF_bias_y = zeros(n_points,n);
UKF_bias_z = zeros(n_points,n);


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
for i = 1:n
    %b) Gyrometer 
%     sat.sensors.gyro_sigma=0.2e-04; %to test
%     sat.sensors.gyro_bias=0.160e-05; %to test
    
    sat.sensors.gyro_sigma = (2.620e-04)*0.5 +rand*(2.620e-04); %to test
    sat.sensors.gyro_bias=(1.160e-04)*0.5 +rand*(1.160e-04); %to test   
    
    PSDgyro=sat.sensors.gyro_sigma^2*Ts;
    PSDbias=sat.sensors.gyro_bias^2*Ts;
    
    b_offset=sat.sensors.gyro_bias;
    b_offset2=-1e-2;    %for the triangular signal
    
    
    GYRO_sigma(i) = sat.sensors.gyro_sigma;
    Gyro_bias(i) = sat.sensors.gyro_bias;
    
    % run the simulink 
    simResults = sim('UKFsim.slx');
    
    %save the data
    UKF_angle_error_rate_x(:,i) = simResults.angle_error_rate.Data(:,1);
    UKF_angle_error_rate_y(:,i) = simResults.angle_error_rate.Data(:,2);
    UKF_angle_error_rate_z(:,i) = simResults.angle_error_rate.Data(:,3);

    UKF_angular_velocity_error_rate_x(:,i) = simResults.angular_velocities_error_rate.Data(:,1);
    UKF_angular_velocity_error_rate_y(:,i) = simResults.angular_velocities_error_rate.Data(:,2);
    UKF_angular_velocity_error_rate_z(:,i) = simResults.angular_velocities_error_rate.Data(:,3);
    
%     UKF_angle_estimation_error_x(:,i) = simResults.angle_real_vs_estimated.Data(:,1);
%     UKF_angle_estimation_error_y(:,i) = simResults.angle_real_vs_estimated.Data(:,2);
%     UKF_angle_estimation_error_z(:,i) = simResults.angle_real_vs_estimated.Data(:,3);

    UKF_bias_x(:,i) = simResults.gyro_bias.Data(1,:);
    UKF_bias_y(:,i) = simResults.gyro_bias.Data(2,:);
    UKF_bias_z(:,i) = simResults.gyro_bias.Data(3,:);
    
    N(i) = i;
    

    Q_real_0(:,i) = simResults.q_real.Data(1,:);
    Q_real_1(:,i) = simResults.q_real.Data(2,:);
    Q_real_2(:,i) = simResults.q_real.Data(3,:);
    Q_real_3(:,i) = simResults.q_real.Data(4,:);

    Q_estimated_0(:,i) = simResults.q_estimated.Data(:,1);
    Q_estimated_1(:,i) = simResults.q_estimated.Data(:,2);
    Q_estimated_2(:,i) = simResults.q_estimated.Data(:,3);
    Q_estimated_3(:,i) = simResults.q_estimated.Data(:,4);


    Q_reference_0(:,i) = simResults.q_ref.Data(1,:);
    Q_reference_1(:,i) = simResults.q_ref.Data(2,:);
    Q_reference_2(:,i) = simResults.q_ref.Data(3,:);
    Q_reference_3(:,i) = simResults.q_ref.Data(4,:);   
    
    Angular_velocity_real_x(:,i) = rad2deg(simResults.w_real.Data(:,1));
    Angular_velocity_real_y(:,i) = rad2deg(simResults.w_real.Data(:,2));
    Angular_velocity_real_z(:,i) = rad2deg(simResults.w_real.Data(:,3));

    Angular_velocity_estimated_x(:,i) = rad2deg(simResults.w_estimated.Data(:,1));
    Angular_velocity_estimated_y(:,i) = rad2deg(simResults.w_estimated.Data(:,2));
    Angular_velocity_estimated_z(:,i) = rad2deg(simResults.w_estimated.Data(:,3));

    Angular_velocity_reference_x(:,i) = rad2deg(simResults.w_ref.Data(:,1));
    Angular_velocity_reference_y(:,i) = rad2deg(simResults.w_ref.Data(:,2));
    Angular_velocity_reference_z(:,i) = rad2deg(simResults.w_ref.Data(:,3));
    
    Angular_velocity_real_vs_estimated_x(:,i) = Angular_velocity_real_x(:,i) - Angular_velocity_estimated_x(:,i);
    Angular_velocity_real_vs_estimated_y(:,i) = Angular_velocity_real_y(:,i) - Angular_velocity_estimated_y(:,i);
    Angular_velocity_real_vs_estimated_z(:,i) = Angular_velocity_real_z(:,i) - Angular_velocity_estimated_z(:,i);

 end

%% Data processing
UKF_average_magnitude_angle_error = zeros(n,3);

UKF_average_magnitude_angular_velocities_error = zeros(n,3);

UKF_average_magnitude_angle_estimation_error = zeros(n,3);

UKF_average_magnitude_bias = zeros(n,3);


Attitude_average_mangnitude_real_vs_estimated = zeros(n,3);

Angular_velocity_average_mangnitude_real_vs_estimated = zeros(n,3);

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
    UKF_average_magnitude_angle_error(k,1) = sqrt(mean(UKF_angle_error_rate_x(:,k).^2 ));
    UKF_average_magnitude_angle_error(k,2) = sqrt(mean(UKF_angle_error_rate_y(:,k).^2 ));
    UKF_average_magnitude_angle_error(k,3) = sqrt(mean(UKF_angle_error_rate_z(:,k).^2 ));

    %Compute the average magnitude (RMS) of the angular velocities error
    UKF_average_magnitude_angular_velocities_error(k,1) = sqrt(mean(UKF_angular_velocity_error_rate_x(:,k).^2 ));
    UKF_average_magnitude_angular_velocities_error(k,2) = sqrt(mean(UKF_angular_velocity_error_rate_y(:,k).^2 ));
    UKF_average_magnitude_angular_velocities_error(k,3) = sqrt(mean(UKF_angular_velocity_error_rate_z(:,k).^2 ));
    

     %Compute the average magnitude (RMS) of the angle estimation error
    UKF_average_magnitude_bias(k,1) = sqrt(mean(UKF_bias_x(:,k).^2 ));
    UKF_average_magnitude_bias(k,2) = sqrt(mean(UKF_bias_y(:,k).^2 ));
    UKF_average_magnitude_bias(k,3) = sqrt(mean(UKF_bias_z(:,k).^2 ));   

    Attitude_real_vs_estimated_x(:,k) = Attitude_real_x(:,k) - Attitude_estimated_x(:,k);
    Attitude_real_vs_estimated_y(:,k) = Attitude_real_y(:,k) - Attitude_estimated_y(:,k);
    Attitude_real_vs_estimated_z(:,k) = Attitude_real_z(:,k) - Attitude_estimated_z(:,k);


    %Compute the average magnitude (RMS) real attitude vs estimated
    Attitude_average_mangnitude_real_vs_estimated(k,1) = sqrt(mean(Attitude_real_vs_estimated_x(:,k).^2 ));
    Attitude_average_mangnitude_real_vs_estimated(k,2) = sqrt(mean(Attitude_real_vs_estimated_y(:,k).^2 ));
    Attitude_average_mangnitude_real_vs_estimated(k,3) = sqrt(mean(Attitude_real_vs_estimated_z(:,k).^2 ));
    
    
    %Compute the average magnitude (RMS) real Angular_velocity vs estimated
    Angular_velocity_average_mangnitude_real_vs_estimated(k,1) = sqrt(mean(Angular_velocity_real_vs_estimated_x(:,k).^2 ));
    Angular_velocity_average_mangnitude_real_vs_estimated(k,2) = sqrt(mean(Angular_velocity_real_vs_estimated_y(:,k).^2 ));
    Angular_velocity_average_mangnitude_real_vs_estimated(k,3) = sqrt(mean(Angular_velocity_real_vs_estimated_z(:,k).^2 ));
    
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
save('UKF_Monte_Carlo_n100.mat');