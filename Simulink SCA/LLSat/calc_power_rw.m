function [power_in_mW] = calc_power_rw(torqueN,ang_vel)
%Calculates the power consumption for one reaction wheel
%   Inputs are:
%    * Torque in N 
%    * Angular speed in rad/s
%   Outputs are:
%    * Power consumption in mW
torque = torqueN * 1000; %calculates the torque in mNm
rpm = ang_vel*60/(2*pi);          %calculates the rpm

torque0=0.4:-0.4/6:0;
b = [276.8603  214.4791   98.2042   76.8537   75.2902   63.3787   62.3787];
m = [0.0814    0.0676    0.0669    0.0559    0.0426    0.0330    0.0228];

bint = interp1(torque0,b,torque);
mint = interp1(torque0,m,torque);
power_in_mW = mint*rpm + bint;
end