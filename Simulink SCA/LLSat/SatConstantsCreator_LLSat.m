load('SatConstant_LLSat.mat')

% inertia, %1 g (mm^2) = 1.0 × 10-9 kg (m^2)
%X=Z, Y=-Y, +Z=+X
% Izz = 5.124E+07 * 1E-9;
% Izy = 16989.818 * 1E-9;
% Izx = 3.075E+05 * 1E-9;
% Iyz = 16989.818 * 1E-9;
% Iyy = 5.158E+07 * 1E-9;
% Iyx = -1.440E+05 * 1E-9;
% Ixz = 3.075E+05 * 1E-9;
% Ixy = -1.440E+05 * 1E-9;
% Ixx = 8.052E+06 * 1E-9;
sat.inertia = [Ixx,Ixy,Ixz;Iyx,Iyy,Iyz;Izx,Izy,Izz];

% mass [kg]
sat.mass = 5;

% bdot_c
%sat.bdot_c = 10E6 %by default?

% CoG deployed [m]
% X axis = 0.066 mm
% Y axis = -0.288 mm
% Z axis = 1.339 mm
sat.CoG = [0.1E-3 -0.3E-3 1.4E-3];


% residual_dipole [A*m^2]
sat.residual_dipole = [0.1;0.1;0.1];

%wheel
sat.wheel.max_torque = 0.2E-3;      %0.2 mNM, max is 0.4mNM (datasheet)
sat.wheel.max_speed = 18000/60*2*pi;%18000 rpm
sat.wheel.inertia = 1.07E-06;       %1,07E-06 kgm^2
sat.wheel.max_sigma = 2E-3;         %max momentum storage 2mNMs
%sat.wheel.repartition_matrix_3RW = eye(3);
%sat.wheel.number = 4;
%sat.wheel.repartition_matrix_4RW
%sat.wheel.repartition_matrix_4RW_inverse
%sat.wheel.N = [0.5;0.5;0.5;0.5];

%mag_t
sat.mag_t.max_mm1 = 0.1522;
sat.mag_t.max_mm2 = 0.1522;
sat.mag_t.max_mm3 = 0.1522;

%thruster
sat.thruster.force=0.475; % Force of thruster in [N]
sat.thruster.d=1*pi/180;           %deviation in rads of the thruster wrt to x axis in plane XY
sat.thruster.alpha=80*pi/180;       %rotation of thrust deviation around x axis
    sat.thruster.R=[ 1    0            0;...
        0    cos(sat.thruster.alpha)  -sin(sat.thruster.alpha);...
        0    sin(sat.thruster.alpha)   cos(sat.thruster.alpha)];
sat.thruster.dir=sat.thruster.R*[cos(sat.thruster.d);sin(sat.thruster.d);0];   %Direction of thrust (unit vector)
%Thruster applied point in BRF:
sat.thruster.x_force_thruster=150;    %Distance from the geometric center x axis in [mm]
sat.thruster.y_force_thruster=0;      %Distance from the geometric center y axis in [mm]
sat.thruster.z_force_thruster=0;      %Distance from the geometric center z axis in [mm]
sat.thruster.point=[-sat.thruster.x_force_thruster;sat.thruster.y_force_thruster;sat.thruster.z_force_thruster]/1e3;    
                            %Distance from the geometric center (BRF) in [m]
                            %Thruster located in "back side" of S/C (-X)
%other parameters for thruster firing
sat.thruster.duration=3;            %duration of the thrust in [seconds]
sat.thruster.wait=3*90*60;          %waiting time between thrusts in [s]
sat.thruster.firstimpulse=4000;     %first thrust after start sim in [s]
                                    %needs to be less than waiting time
sat.thruster.Nfirings=2;            %number of thrust firings

%PID
%not modified for the moment

%sensors
%not modified for the moment