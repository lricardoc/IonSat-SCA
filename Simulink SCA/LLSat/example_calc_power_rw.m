%Calculate power consumption
% Specify the path to your CSV file
filename = 'pointsRWpower.csv';
% Read the data from the CSV file into a matrix
xypoints = readmatrix(filename);
% Display the imported points
disp('Imported points:');
disp(xypoints);

%Get 7 eqs + 1
for i=1:7
    [m(i), b(i), ~] = makeEquationFrom2Points(xypoints(2*(i-1)+1,1),xypoints(2*(i-1)+1,2),xypoints(2*(i-1)+2,1),xypoints(2*(i-1)+2,2));
end
%modifying the value of m to make it ascending
power.b = b;
power.m = m;
power.b(4) = 76.8537;
power.b(6) = 63.3787;
power.rpm=0:100:18000;
for i=1:7
    power.mW(i,:)=power.m(i)*power.rpm+power.b(i);
end
figure()
plot(power.rpm, power.mW(1,:)) %for 0.4 mNm
hold on
plot(power.rpm, power.mW(2,:)) %for 0.33 mNm
plot(power.rpm, power.mW(3,:)) %for 0.2667
plot(power.rpm, power.mW(4,:)) %for 0.2
plot(power.rpm, power.mW(5,:)) %etc
plot(power.rpm, power.mW(6,:))
plot(power.rpm, power.mW(7,:))
grid on
torque=0.4:-0.4/6:0;


%example:
tau=0.3;
bint = interp1(torque,b,tau);
mint = interp1(torque,m,tau);
rpm=10000;
milliwatt = mint*rpm + bint;

%with function
%[power_in_mW] = calc_power_rw(torqueN,ang_vel)
[power_in_mW] = calc_power_rw(0.3E-3,1047.1975511965977);