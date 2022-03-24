clear

d=pi/180;
Time= 50 * 60; %50 minutes

d_alpha=3*pi/180;
a=0:d_alpha:2*pi;

x=0;
y=0.00381;
z=0.00823;

% y=0.0041;
% z=0.0085;
% 
% a=0;

MMSx=(cos(a)*sin(d)*z-sin(a)*sin(d)*y)*Time*0.75;
MMSy=(sin(a)*sin(d)*0.171-cos(d)*z)*Time*0.75;
MMSz=(cos(d)*y-cos(a)*sin(d)*0.171)*Time*0.75;

figure()
set(gcf,'color','w');
    subplot(3,1,1)
    plot(a*180/pi,MMSx);
    grid on
    subplot(3,1,2)
    plot(a*180/pi,MMSy);
    grid on
    subplot(3,1,3)
    plot(a*180/pi,MMSz);
    grid on
    
%% Calc 2
clear

d=pi/180;           %deviation in rads of the thruster wrt to x axis in plane XY
Time= 50 * 60;      %time in seconds (50 minutes)

%the deviation is of d=1 degree
F=0.75*[cos(d);sin(d);0]*1e-3;
y=3.81;             %Distance from the geometric center y axis in mm
z=8.23;             %Distance from the geometric center z axis in mm
D=[171;y;z]/1e3;    %Distance from the geometric center in m

%the rotation of a degrees the force around x axis 
step_angle=1;
a=0;F_rot=[0;0;0];MMS=[0;0;0];
for i=1:360/step_angle+1
    alpha=step_angle*(i-1)*pi/180;
    a(i)=alpha*180/pi;
    R=[ 1    0            0;...
        0    cos(alpha)  -sin(alpha);...
        0    sin(alpha)   cos(alpha)];
F_rot(:,i)=R*F;
MMS(:,i)=cross(F_rot(:,i),D)*1e3*Time;
end


figure()
set(gcf,'color','w');
plot(a,F_rot(2,:))
hold on
plot(a,F_rot(3,:))
%plot(a,F_rot(1,:))
grid on

MMSx=MMS(1,:);
MMSy=MMS(2,:);
MMSz=MMS(3,:);

figure()
set(gcf,'color','w');
    subplot(3,1,1)
    plot(a,MMSx);
    grid on
    subplot(3,1,2)
    plot(a,MMSy);
    grid on
    subplot(3,1,3)
    plot(a,MMSz);
    grid on
