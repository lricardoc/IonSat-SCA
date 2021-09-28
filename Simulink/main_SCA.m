%% Clear workspace and figures
clear all
close all
clc

%% Loading the Data
%Loads the necessary physical variables describing the satellite and its
%components
load('SatConstants.mat')
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
%Orbit simulation
%Initialisation of Keplerian parameters
% a = 6678; %semimajor axis (km)
% e = 0.001; %eccentricity
% i = 51.6; %inclination (degrees)
% O = 300.5; %Right ascension of the right ascending node (degrees) %max 197, min 300.5 %181
% o = 0; %Argument of the perigee (degrees)                      %max 90, min 0      %90
% nu = 0; %True anomaly (degrees)
% 
% %Initialisation of date
% year = 2020;
% month = 1;
% day = 1;
% hours = 0;
% minutes = 0;
% seconds = 0;
% 
% %Simulation time parameters
% N_orbits = 2;
% %Torbit=90*60;       %1 orbit approx. 90 minutes
% Torbit=2*pi*sqrt(a^3/(3.986004418E5));

%% Declare variables
%time step based on gyro sampling frequency: 
TimeStep = 1;

%Time spent performing the simulation in seconds (one orbit is 5400 s):
t_sim = 10800;
OrbitSize = 5400;

%Determine the order of approximation for IGRF model of the earth's
%magnetic field nmax (must be equal or less than 13)
nmax = 2;

%Determine whether you are using full IGRF model or dipole approximation
%1 for true, 0 for false
%Use_IGRF = 0; 
Use_IGRF = 1; 
%Determine whether you are using full IGRF model or dipole approximation
%for the Kalman Filter
%1 for true, 0 for false
%Use_IGRF_KF = 0; 
Use_IGRF_KF = 1; 


%Decide whether you'd like to use control or not
%A value of 1 (True) entails that you'd like to simulate a realistic model
%for the control system, and 0 (False) means you assume ideal no control conditions
Enable_control = 0;

%Enable whether the sensors are considering eclipses or not
Enable_eclipse = 0;

%Global time step
global dt
%Sampling times:
Tss=1;         %sampling time for MTM, FSS, CSS could be 1 not 0.5
Ts=1;          %sampling time for Gyro

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

%% Calculate the pseudo inverse matrix
Pinv_RW_repartition

%% Load the simulink model
IonSatSimulationR2019A

%% Load the simulink model
load_system("IonSatSimulationR2019A.slx")

%% Run the simulink 
simOut = sim("IonSatSimulation1");