%% MODE Manager
%Initialisation of date
date.year = 2024;
date.month = 3;
date.day = 21;
date.hours = 0;
date.minutes = 0;
date.seconds = 0;

%Get initial position and velocity
[orbit.r,orbit.v] = orb2rv_init(orbit.a,orbit.e,orbit.i*pi/180,orbit.O*pi/180,orbit.o*pi/180,orbit.nu*pi/180,0,0,0);
%Get Sun initial direction
%Implement the position of the Sun with respect to the Earth for initialization date with DE405.
orbit.sun_ECI_0 = (planetEphemeris(juliandate(date.year,date.month,date.day),'Earth','Sun'))';
orbit.sun_ECI_0 = orbit.sun_ECI_0/norm(orbit.sun_ECI_0);   %get unit vector

%POINTING MODE
MODE = 2;   

if MODE == 1 %"orbital" 
    % Reference quaternion is aligned with ORF. 
    x_sa = orbit.v/norm(orbit.v);
    z_sa = orbit.r/norm(orbit.r);
    y_sa = -cross(x_sa,z_sa);
    DCM_orb = horzcat(x_sa,y_sa,z_sa);
    q_s2o = dcm2quat(DCM_orb);
    att.q_i2r = (quatinv(q_s2o))';
end
if MODE == 2 %"sun-aero" 
    % Reference quaternion is such that x is aligned with velocity 
    % and z is aligned as best as possible with the sun direction to maximize the power generation 
    x_sa = orbit.v/norm(orbit.v);
    z_sa = orbit.sun_ECI_0 - dot(x_sa,orbit.sun_ECI_0)*x_sa;
    z_sa = z_sa/norm(z_sa);
    y_sa = cross(z_sa,x_sa);
    DCM_sa = horzcat(x_sa,y_sa,z_sa);
    q_s2i = dcm2quat(DCM_sa);
    att.q_i2r = (quatinv(q_s2i))';
end
if MODE == 3 %"sun pointing" 
    % Reference quaternion is such that z is aligned with with the sun direction to maximize the power generation 
    % and x is aligned as best as possible with the velocity direction.
    z_sa = orbit.sun_ECI_0/norm(orbit.sun_ECI_0);
    x_sa = dot(orbit.v,z_sa)*z_sa - orbit.v;
    x_sa = x_sa/norm(x_sa);
    y_sa = cross(z_sa,x_sa);
    DCM_sa=horzcat(x_sa,y_sa,z_sa);
    q_s2i = dcm2quat(DCM_sa);
    att.q_i2r = (quatinv(q_s2i))';
end
if MODE == 4   % "aero-drag" 
    % Reference quaternion is similar than in Case 1, but rotated 90° along the y axis
    x_sa = orbit.r/norm(orbit.r);
    z_sa = orbit.v/norm(orbit.v);
    y_sa = cross(x_sa,z_sa);
    DCM_sa=horzcat(x_sa,y_sa,z_sa);  
    q_s2i = dcm2quat(DCM_sa);
    att.q_i2r = (quatinv(q_s2i))';
end
if MODE == 5 %"retrogade firing" 
    % Reference quaternion is similar than in Case 1, but rotated 180° along the z axis
    x_sa = - orbit.v/norm(orbit.v);
    z_sa = orbit.sun_ECI_0 - dot(x_sa,orbit.sun_ECI_0)*x_sa;
    z_sa = z_sa/norm(z_sa);
    y_sa = cross(z_sa,x_sa);
    DCM_sa = horzcat(x_sa,y_sa,z_sa);
    q_s2i = dcm2quat(DCM_sa);
    att.q_i2r = (quatinv(q_s2i))';
end
if MODE == 6
    %Reference quaternion is static [1 0 0 0], but this time is the only case where the state of the B-dot is enabled
    att.q_i2r = [1 0 0 0];
end
if MODE == 7
    %Reference quaternion is a custom quaternion that has to be defined by the user as an input to the mission block
    prompt = "What is the desired quaternion value? ";
    att.q_i2r = input(prompt);
end
if MODE == 8  %"sun-drag" 
    % Reference quaternion is such that z is aligned with the velocity direction to maximize the drag 
    % and x is aligned as best as possible with the velocity direction. 
    z_sa = orbit.sun_ECI_0/norm(orbit.sun_ECI_0);
    nadir = - orbit.r/norm(orbit.r);
    x_sa = dot(nadir,z_sa)*z_sa - nadir;
    x_sa = x_sa/norm(x_sa);
    y_sa = cross(z_sa,x_sa);
    DCM_sa=horzcat(x_sa,y_sa,z_sa);
    q_s2i = dcm2quat(DCM_sa);
    att.q_i2r = (quatinv(q_s2i))';
end