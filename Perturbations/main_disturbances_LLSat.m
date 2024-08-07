clear
close

%Initialisation of Keplerian parameters
alt = 275;              %Altitude.....................(Km)							
ecc = 0.001;            %Eccentricity											    
inc =  98;              %Inclination..................(deg)		
RAAN = 0;               %Right Asc. of Ascending Node.(deg)	
w = 90;                 %Argument of perigee..........(deg)	
nu = 00;                %Satellite position...........(deg)		

%beta angle  ~9 deg. 21/3, i=98, O=10, 
%beta angle ~29 deg. 21/3, i=98, O=30,
%beta angle ~48 deg. 21/3, i=98, O=50,  

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
date.month = 3;
date.day = 21;
date.hours = 0;
date.minutes = 0;
date.seconds = 0;

angle_roll = pi/4;

%Load Satellite Constants
load('SatConstants.mat')

%%% MAGNETIC FIELD MODELS - GET PROPER IGRF COEFFICIENTS %%%
date_IGRF = [date.year,date.month,date.day];
%"REAL" Magnetic Field
%Determine whether you are using full IGRF model or dipole approximation
%1 for full model, 0 for dipole approximation
Use_IGRF = 0; 
nmax = 2;

%%% Model of Perturbation Torques
%Magnetic Torque Residual dipole
sat.residual_dipole=[0.1;0.1;0.1]; %residual magnetic dipole in [A*m^2]
%Aerodynamic Torque Constants
%load('LOAS.mat')`
load('LLSat_3U.mat');
LLSataero.T = SNAP_aeromodel.T;
LLSataero.El = SNAP_aeromodel.Az;
LLSataero.Az = SNAP_aeromodel.El;
LLSataero.Tx = SNAP_aeromodel.T_x;
LLSataero.Ty = SNAP_aeromodel.T_y;
LLSataero.Tz = SNAP_aeromodel.T_z;
LLSataero.av_density_vs_alt = SNAP_aeromodel.av_density_vs_alt;
LLSataero.alt_range = SNAP_aeromodel.alt_range;
LLSataero.A_drag = SNAP_aeromodel.A_drag;
%Solar radiation pressure torque
F_s = 1367;     %solar constant in [W/m^2]
c = 299792458;  %light speed in [m/s]
q = 0.6;        %reflectance factor 
%q = (400/((alt+6378)/6378)^2)/F_s;        %reflectance factor 
Cd = 2.5;

%% Open Model
distmodel_v6_LLSat
%Power=sim('EPS_orbit_v3a');
%DOD = getdatasamples(E.DOD,(1:length(time)));
%% plot and analyze drag surface
time = out.drag.Time;
A_drag = squeeze(out.drag.Data);
figure()
    plot(time/3600, A_drag)
    hold on
    grid on
    legend('Drag Surface (m^2)')
    xlabel('Time (hours)')
    ylabel('Area (m^2)')
A_drag_mean = mean(A_drag);
A_drag_median = median(A_drag);