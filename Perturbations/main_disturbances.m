clear
close

alt = 300;              %Altitude.....................(Km)							
ecc = 0.001;            %Eccentricity											    
inc =  51.6;            %Inclination..................(deg)		
RAAN = 30;              %Right Asc. of Ascending Node.(deg)	
w = 25;                 %Argument of perigee..........(deg)	
nu = 75;                %Satellite position...........(deg)							

inc = rad2deg(inc);
RAAN = rad2deg(RAAN);
w = rad2deg(w);
nu = rad2deg(nu);

%time
TimeStep = 1;        %fixed-step size in solver, Default time step=0.25
%TimeStep = 0.5;        %fixed-step size in solver, Default time step=0.25
Torbit=2*pi*sqrt((alt+6378.1)^3/(3.986004418E5));
N_orbits = 2;           %number of orbits to be simulated
%Time spent performing the simulation in seconds (one orbit is ~5400 s):
t_sim = N_orbits*Torbit;

%Initialisation of date
date.year = 2022;
date.month = 1;
date.day = 1;
date.hours = 0;
date.minutes = 0;
date.seconds = 0;

%Load Satellite Constants
load('SatConstants.mat')

%Aerodynamic Torque Constants
%load('LOAS.mat')
load('IonSat_6U.mat');
IonSataero.T = SNAP_aeromodel.T;
IonSataero.pitch = SNAP_aeromodel.pitch;
IonSataero.roll = SNAP_aeromodel.roll;
IonSataero.av_density_vs_alt = SNAP_aeromodel.av_density_vs_alt;
IonSataero.alt_range = SNAP_aeromodel.alt_range;

distmodel_v3

