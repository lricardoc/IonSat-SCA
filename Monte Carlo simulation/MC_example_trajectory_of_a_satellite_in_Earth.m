clear
close
% Monte Carlo simulation for satellite trajectory
% Parameters
mu = 3.986e14; % gravitational parameter of Earth
R = 6.371e6; % Earth radius
alt = 500e3; % altitude of satellite
v0 = sqrt(mu/(R+alt)); % initial velocity of satellite
n = 1000; % number of simulations

% Define initial conditions
x0 = R+alt; % initial position
y0 = 0;
z0 = 0;
vx0 = 0; % initial velocity
vy0 = v0;
vz0 = 0;

% Define time vector
tspan = [0 2*pi*sqrt((R+alt)^3/mu)];

% Define ODE system
odefun = @(t,x) [x(4); x(5); x(6); -mu*x(1)/norm(x(1:3))^3; -mu*x(2)/norm(x(1:3))^3; -mu*x(3)/norm(x(1:3))^3];

% Solve ODE system
[t,x] = ode45(odefun, tspan, [x0 y0 z0 vx0 vy0 vz0]);

% Plot nominal trajectory
figure
plot3(x(:,1), x(:,2), x(:,3))
grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Nominal Satellite Trajectory')

% Monte Carlo simulation
figure
hold on
for i = 1:n
    % Add noise to initial conditions
    x0_noisy = x0 + randn*1000;
    y0_noisy = y0 + randn*1000;
    z0_noisy = z0 + randn*1000;
    vx0_noisy = vx0 + randn*0.1;
    vy0_noisy = vy0 + randn*0.1;
    vz0_noisy = vz0 + randn*0.1;
    
    % Solve ODE system with noisy initial conditions
    [t,x_noisy] = ode45(odefun, tspan, [x0_noisy y0_noisy z0_noisy vx0_noisy vy0_noisy vz0_noisy]);
    
    % Plot trajectory
    plot3(x_noisy(:,1), x_noisy(:,2), x_noisy(:,3))
end
grid on
xlabel('x')
ylabel('y')
zlabel('z')
title('Monte Carlo Satellite Trajectories')