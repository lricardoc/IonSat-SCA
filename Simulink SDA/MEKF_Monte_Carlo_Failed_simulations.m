%%%Script that gives the number of failed simulations function of the criteria

n = length(MEKF_angle_error_rate_x(1,:)); %number of simulations
t_sim = length(MEKF_angle_error_rate_x(:,1)); %duration of the simulations
t_steady_state = round(t_sim*0.85);

%% Criteria 1 : on the angle error rate 
% In the steady-state, the error rate must be between -5° and +5°

%initialise the variables that will count the number of failed simulations
N_failed_C1_x = 0;
N_failed_C1_y = 0;
N_failed_C1_z = 0;

%intitialize the lists that will save the index of the failed simulations
Failed_index_list_C1_x = zeros(1,n);
Failed_index_list_C1_y = zeros(1,n);
Failed_index_list_C1_z = zeros(1,n);

%loop that increments the previous variables
for i = 1:n
    C1_failed_x = 0;
    C1_failed_y = 0;
    C1_failed_z = 0;
    %if during the last 1000 seconds of simulation the criteria isn't validated, then it failed
    for j=t_steady_state:t_sim
        if (mean(MEKF_angle_error_rate_x(j,i))>5) || (mean(MEKF_angle_error_rate_x(j,i))<-5)
            C1_failed_x = C1_failed_x+1;
        end
        if (mean(MEKF_angle_error_rate_y(j,i))>5) || (mean(MEKF_angle_error_rate_y(j,i))<-5)
            C1_failed_y = C1_failed_y+1;
        end
        if (mean(MEKF_angle_error_rate_z(j,i))>5) || (mean(MEKF_angle_error_rate_z(j,i))<-5)
            C1_failed_z = C1_failed_z+1;
        end
    end
    if C1_failed_x > 0
       N_failed_C1_x = N_failed_C1_x + 1;
       Failed_index_list_C1_x(N_failed_C1_x)=i;
    end
    if C1_failed_y > 0
       N_failed_C1_y = N_failed_C1_y + 1;
       Failed_index_list_C1_y(N_failed_C1_y)=i;
    end
    if C1_failed_z > 1
       N_failed_C1_z = N_failed_C1_z + 1;
       Failed_index_list_C1_z(N_failed_C1_z)=i;
    end
end

%Remove the unnecesarry part of the list
Failed_index_list_C1_x(N_failed_C1_x+1:n)=[];
Failed_index_list_C1_y(N_failed_C1_y+1:n)=[];
Failed_index_list_C1_z(N_failed_C1_z+1:n)=[];

%Compute the percentage of failed simulations
Failed_percentage_C1_x = 100*N_failed_C1_x/n;
Failed_percentage_C1_y = 100*N_failed_C1_y/n;
Failed_percentage_C1_z = 100*N_failed_C1_z/n;

Failed_percentage_average_C1 = (Failed_percentage_C1_x + Failed_percentage_C1_y + Failed_percentage_C1_z)/3;

%% Criteria 1bis : on the angle error rate, in caseboth MEKF and MEKF fails C1 at 100%
% In the steady-state, the error rate must be between -15° and +15°

%initialise the variables that will count the number of failed simulations
N_failed_C1bis_x = 0;
N_failed_C1bis_y = 0;
N_failed_C1bis_z = 0;

%intitialize the lists that will save the index of the failed simulations
Failed_index_list_C1bis_x = zeros(1,n);
Failed_index_list_C1bis_y = zeros(1,n);
Failed_index_list_C1bis_z = zeros(1,n);

%loop that increments the previous variables
for i = 1:n
    C1bis_failed_x = 0;
    C1bis_failed_y = 0;
    C1bis_failed_z = 0;
    %if during the last 1000 seconds of simulation the criteria isn't validated, then it failed
    for j=t_steady_state:t_sim
        if (mean(MEKF_angle_error_rate_x(j,i))>15) || (mean(MEKF_angle_error_rate_x(j,i))<-15)
            C1bis_failed_x = C1bis_failed_x+1;
        end
        if (mean(MEKF_angle_error_rate_y(j,i))>15) || (mean(MEKF_angle_error_rate_y(j,i))<-15)
            C1bis_failed_y = C1bis_failed_y+1;
        end
        if (mean(MEKF_angle_error_rate_z(j,i))>15) || (mean(MEKF_angle_error_rate_z(j,i))<-15)
            C1bis_failed_z = C1bis_failed_z+1;
        end
    end
    if C1bis_failed_x > 0
       N_failed_C1bis_x = N_failed_C1bis_x + 1;
       Failed_index_list_C1bis_x(N_failed_C1bis_x)=i;
    end
    if C1bis_failed_y > 0
       N_failed_C1bis_y = N_failed_C1bis_y + 1;
       Failed_index_list_C1bis_y(N_failed_C1bis_y)=i;
    end
    if C1bis_failed_z > 1
       N_failed_C1bis_z = N_failed_C1bis_z + 1;
       Failed_index_list_C1bis_z(N_failed_C1bis_z)=i;
    end
end

%Remove the unnecesarry part of the list
Failed_index_list_C1bis_x(N_failed_C1bis_x+1:n)=[];
Failed_index_list_C1bis_y(N_failed_C1bis_y+1:n)=[];
Failed_index_list_C1bis_z(N_failed_C1bis_z+1:n)=[];

%Compute the percentage of failed simulations
Failed_percentage_C1bis_x = 100*N_failed_C1bis_x/n;
Failed_percentage_C1bis_y = 100*N_failed_C1bis_y/n;
Failed_percentage_C1bis_z = 100*N_failed_C1bis_z/n;

Failed_percentage_average_C1bis = (Failed_percentage_C1bis_x + Failed_percentage_C1bis_y + Failed_percentage_C1bis_z)/3;

%% Criteria 2 : on the angular velocity error rate
%In the steady-state, the error rate must be between -0,5°/s and +0,5°/s

%initialise the variables that will count the number of failed simulations
N_failed_C2_x = 0;
N_failed_C2_y = 0;
N_failed_C2_z = 0;

%intitialize the lists that will save the index of the failed simulations
Failed_index_list_C2_x = zeros(1,n);
Failed_index_list_C2_y = zeros(1,n);
Failed_index_list_C2_z = zeros(1,n);

%loop that increments the previous variables
for i = 1:n
    C2_failed_x = 0;
    C2_failed_y = 0;
    C2_failed_z = 0;
    %if during the last seconds of simulation the criteria isn't validated, then it failed
    for j=t_steady_state:t_sim
        if (mean(MEKF_angular_velocity_error_rate_x(j,i))>0.5) || (mean(MEKF_angular_velocity_error_rate_x(j,i))<-0.5)
            C2_failed_x = C2_failed_x+1;
        end
        if (mean(MEKF_angular_velocity_error_rate_y(j,i))>0.5) || (mean(MEKF_angular_velocity_error_rate_y(j,i))<-0.5)
            C2_failed_y = C2_failed_y+1;
        end
        if (mean(MEKF_angular_velocity_error_rate_z(j,i))>0.5) || (mean(MEKF_angular_velocity_error_rate_z(j,i))<-0.5)
            C2_failed_z = C2_failed_z+1;
        end
    end
    if C2_failed_x > 0
       N_failed_C2_x = N_failed_C2_x + 1;
       Failed_index_list_C2_x(N_failed_C2_x)=i;
    end
    if C2_failed_y > 0
       N_failed_C2_y = N_failed_C2_y + 1;
       Failed_index_list_C2_y(N_failed_C2_y)=i;
    end
    if C2_failed_z > 1
       N_failed_C2_z = N_failed_C2_z + 1;
       Failed_index_list_C2_z(N_failed_C2_z)=i;
    end
end

%Remove the unnecesarry part of the list
Failed_index_list_C2_x(N_failed_C2_x+1:n)=[];
Failed_index_list_C2_y(N_failed_C2_y+1:n)=[];
Failed_index_list_C2_z(N_failed_C2_z+1:n)=[];

%Compute the percentage of failed simulations
Failed_percentage_C2_x = 100*N_failed_C2_x/n;
Failed_percentage_C2_y = 100*N_failed_C2_y/n;
Failed_percentage_C2_z = 100*N_failed_C2_z/n;

Failed_percentage_average_C2 = (Failed_percentage_C2_x + Failed_percentage_C2_y + Failed_percentage_C2_z)/3;

%% Criteria 2bis : on the angular velocity error rate, in case both MEKF and MEKF fails C2 at 100%
%In the steady-state, the error rate must be between -1°/s and +1°/s

%initialise the variables that will count the number of failed simulations
N_failed_C2bis_x = 0;
N_failed_C2bis_y = 0;
N_failed_C2bis_z = 0;

%intitialize the lists that will save the index of the failed simulations
Failed_index_list_C2bis_x = zeros(1,n);
Failed_index_list_C2bis_y = zeros(1,n);
Failed_index_list_C2bis_z = zeros(1,n);

%loop that increments the previous variables
for i = 1:n
    C2bis_failed_x = 0;
    C2bis_failed_y = 0;
    C2bis_failed_z = 0;
    %if during the last 1000 seconds of simulation the criteria isn't validated, then it failed
    for j=t_steady_state:t_sim
        if (mean(MEKF_angular_velocity_error_rate_x(j,i))>1.5) || (mean(MEKF_angular_velocity_error_rate_x(j,i))<-1.5)
            C2bis_failed_x = C2bis_failed_x+1;
        end
        if (mean(MEKF_angular_velocity_error_rate_y(j,i))>1.5) || (mean(MEKF_angular_velocity_error_rate_y(j,i))<-1.5)
            C2bis_failed_y = C2bis_failed_y+1;
        end
        if (mean(MEKF_angular_velocity_error_rate_z(j,i))>1.5) || (mean(MEKF_angular_velocity_error_rate_z(j,i))<-1.5)
            C2bis_failed_z = C2bis_failed_z+1;
        end
    end
    if C2bis_failed_x > 0
       N_failed_C2bis_x = N_failed_C2bis_x + 1;
       Failed_index_list_C2bis_x(N_failed_C2bis_x)=i;
    end
    if C2bis_failed_y > 0
       N_failed_C2bis_y = N_failed_C2bis_y + 1;
       Failed_index_list_C2bis_y(N_failed_C2bis_y)=i;
    end
    if C2bis_failed_z > 1
       N_failed_C2bis_z = N_failed_C2bis_z + 1;
       Failed_index_list_C2bis_z(N_failed_C2bis_z)=i;
    end
end

%Remove the unnecesarry part of the list
Failed_index_list_C2bis_x(N_failed_C2bis_x+1:n)=[];
Failed_index_list_C2bis_y(N_failed_C2bis_y+1:n)=[];
Failed_index_list_C2bis_z(N_failed_C2bis_z+1:n)=[];

%Compute the percentage of failed simulations
Failed_percentage_C2bis_x = 100*N_failed_C2bis_x/n;
Failed_percentage_C2bis_y = 100*N_failed_C2bis_y/n;
Failed_percentage_C2bis_z = 100*N_failed_C2bis_z/n;

Failed_percentage_average_C2bis = (Failed_percentage_C2bis_x + Failed_percentage_C2bis_y + Failed_percentage_C2bis_z)/3;

%% Criteria 5 : on the angle estimator error  
% The uncertainty on the pointing accuracy must be between -3° and +3°
%In other word
% %%Rearange data
% n_point = length(Attitude_real_vs_estimated_x(:,1));
% Attitude_real_vs_estimated_process_x = zeros(n_point,n);
% Attitude_real_vs_estimated_process_y = zeros(n_point,n);
% Attitude_real_vs_estimated_process_z = zeros(n_point,n);
% for i=1:100
%     parfor j=1:n_point
%         if Attitude_real_vs_estimated_x(j,i)>180 
%             Attitude_real_vs_estimated_process_x(j,i)=mod(360,Attitude_real_vs_estimated_x(j,i));
%         elseif Attitude_real_vs_estimated_x(j,i)<-180
%             Attitude_real_vs_estimated_process_x(j,i)=mod(-360,Attitude_real_vs_estimated_x(j,i));
%         else
%             Attitude_real_vs_estimated_process_x(j,i)=Attitude_real_vs_estimated_x(j,i);
%         end    
%         if Attitude_real_vs_estimated_y(j,i)>180 
%             Attitude_real_vs_estimated_process_y(j,i)=mod(360,Attitude_real_vs_estimated_y(j,i));
%         elseif Attitude_real_vs_estimated_y(j,i)<-180
%             Attitude_real_vs_estimated_process_y(j,i)=mod(-360,Attitude_real_vs_estimated_y(j,i));
%         else
%             Attitude_real_vs_estimated_process_y(j,i)=Attitude_real_vs_estimated_y(j,i);
%         end    
%         if Attitude_real_vs_estimated_z(j,i)>180 
%             Attitude_real_vs_estimated_process_z(j,i)=mod(360,Attitude_real_vs_estimated_z(j,i));
%         elseif Attitude_real_vs_estimated_z(j,i)<-180
%             Attitude_real_vs_estimated_process_z(j,i)=mod(-360,Attitude_real_vs_estimated_z(j,i));
%         else
%             Attitude_real_vs_estimated_process_z(j,i)=Attitude_real_vs_estimated_z(j,i);
%         end
%     end
% end

%initialise the variables that will count the number of failed simulations
N_failed_C5_x = 0;
N_failed_C5_y = 0;
N_failed_C5_z = 0;

%intitialize the lists that will save the index of the failed simulations
Failed_index_list_C5_x = zeros(1,n);
Failed_index_list_C5_y = zeros(1,n);
Failed_index_list_C5_z = zeros(1,n);

%loop that increments the previous variables
for i = 1:n
    C5_failed_x = 0;
    C5_failed_y = 0;
    C5_failed_z = 0;
    for j=t_steady_state:t_sim
        if (mean(Attitude_real_vs_estimated_process_x(j,i))>3) || (mean(Attitude_real_vs_estimated_process_x(j,i))<-3)
            C5_failed_x = C5_failed_x+1;
        end
        if (mean(Attitude_real_vs_estimated_process_y(j,i))>3) || (mean(Attitude_real_vs_estimated_process_y(j,i))<-3)
            C5_failed_y = C5_failed_y+1;
        end
        if (mean(Attitude_real_vs_estimated_process_z(j,i))>3) || (mean(Attitude_real_vs_estimated_process_z(j,i))<-3)
            C5_failed_z = C5_failed_z+1;
        end
    end
    if C5_failed_x > 0
       N_failed_C5_x = N_failed_C5_x + 1;
       Failed_index_list_C5_x(N_failed_C5_x)=i;
    end
    if C5_failed_y > 0
       N_failed_C5_y = N_failed_C5_y + 1;
       Failed_index_list_C5_y(N_failed_C5_y)=i;
    end
    if C5_failed_z > 1
       N_failed_C5_z = N_failed_C5_z + 1;
       Failed_index_list_C5_z(N_failed_C5_z)=i;
    end
end

%Remove the unnecesarry part of the list
Failed_index_list_C5_x(N_failed_C5_x+1:n)=[];
Failed_index_list_C5_y(N_failed_C5_y+1:n)=[];
Failed_index_list_C5_z(N_failed_C5_z+1:n)=[];

%Compute the percentage of failed simulations
Failed_percentage_C5_x = 100*N_failed_C5_x/n;
Failed_percentage_C5_y = 100*N_failed_C5_y/n;
Failed_percentage_C5_z = 100*N_failed_C5_z/n;

Failed_percentage_average_C5 = (Failed_percentage_C5_x + Failed_percentage_C5_y + Failed_percentage_C5_z)/3;

%% Criteria 5bis : on the angle error rate, in case both UKF and UKF fails C5 at 100%
% In the steady-state, the error rate must be between -9° and +9°

%initialise the variables that will count the number of failed simulations
N_failed_C5bis_x = 0;
N_failed_C5bis_y = 0;
N_failed_C5bis_z = 0;

%intitialize the lists that will save the index of the failed simulations
Failed_index_list_C5bis_x = zeros(1,n);
Failed_index_list_C5bis_y = zeros(1,n);
Failed_index_list_C5bis_z = zeros(1,n);

%loop that increments the previous variables
for i = 1:n
    C5bis_failed_x = 0;
    C5bis_failed_y = 0;
    C5bis_failed_z = 0;
    for j=t_steady_state:t_sim
        if (mean(Attitude_real_vs_estimated_process_x(j,i))>9) || (mean(Attitude_real_vs_estimated_process_x(j,i))<-9)
            C5bis_failed_x = C5bis_failed_x+1;
        end
        if (mean(Attitude_real_vs_estimated_process_y(j,i))>9) || (mean(Attitude_real_vs_estimated_process_y(j,i))<-9)
            C5bis_failed_y = C5bis_failed_y+1;
        end
        if (mean(Attitude_real_vs_estimated_process_z(j,i))>9) || (mean(Attitude_real_vs_estimated_process_z(j,i))<-9)
            C5bis_failed_z = C5bis_failed_z+1;
        end
    end
    if C5bis_failed_x > 0
       N_failed_C5bis_x = N_failed_C5bis_x + 1;
       Failed_index_list_C5bis_x(N_failed_C5bis_x)=i;
    end
    if C5bis_failed_y > 0
       N_failed_C5bis_y = N_failed_C5bis_y + 1;
       Failed_index_list_C5bis_y(N_failed_C5bis_y)=i;
    end
    if C5bis_failed_z > 1
       N_failed_C5bis_z = N_failed_C5bis_z + 1;
       Failed_index_list_C5bis_z(N_failed_C5bis_z)=i;
    end
end

%Remove the unnecesarry part of the list
Failed_index_list_C5bis_x(N_failed_C5bis_x+1:n)=[];
Failed_index_list_C5bis_y(N_failed_C5bis_y+1:n)=[];
Failed_index_list_C5bis_z(N_failed_C5bis_z+1:n)=[];

%Compute the percentage of failed simulations
Failed_percentage_C5bis_x = 100*N_failed_C5bis_x/n;
Failed_percentage_C5bis_y = 100*N_failed_C5bis_y/n;
Failed_percentage_C5bis_z = 100*N_failed_C5bis_z/n;

Failed_percentage_average_C5bis = (Failed_percentage_C5bis_x + Failed_percentage_C5bis_y + Failed_percentage_C5bis_z)/3;



%% Display the failed simulation percentage 
Criteria = categorical({'C1','C1bis','C2','C2bis','C5','C5bis'});
Criteria = reordercats(Criteria,{'C1','C1bis','C2','C2bis','C5','C5bis'});
Failed_percentage = [Failed_percentage_average_C1;Failed_percentage_average_C1bis;Failed_percentage_average_C2;Failed_percentage_average_C2bis;Failed_percentage_average_C5;Failed_percentage_average_C5bis];
figure;
bar(Criteria,Failed_percentage);
xlabel('Criteria');
ylabel('Percentage of failure');
title(['Percentage of Failed Simulations function of the Criteria']);
grid on;

%% Display data of the failed simulations 
%Get the max number of failed simulations
n_failed_C1_x = length(Failed_index_list_C1_x);
n_failed_C1_y = length(Failed_index_list_C1_y);
n_failed_C1_z = length(Failed_index_list_C1_z);
n_failed_C1 = max([n_failed_C1_x;n_failed_C1_y;n_failed_C1_z]); % max number of C1 failed simlations

n_failed_C2_x = length(Failed_index_list_C2_x);
n_failed_C2_y = length(Failed_index_list_C2_y);
n_failed_C2_z = length(Failed_index_list_C2_z);
n_failed_C2 = max([n_failed_C2_x;n_failed_C2_y;n_failed_C2_z]); % max number of C2 failed simlations

n_failed_C5_x = length(Failed_index_list_C5_x);
n_failed_C5_y = length(Failed_index_list_C5_y);
n_failed_C5_z = length(Failed_index_list_C5_z);
n_failed_C5 = max([n_failed_C5_x;n_failed_C5_y;n_failed_C5_z]); % max number of C5 failed simlations

%Get the index of the failed simulations
Failed_index_list_C1 = []; 
Failed_index_list_C2 = []; 
Failed_index_list_C5 = []; 

if n_failed_C1 == length(Failed_index_list_C1_x)
    Failed_index_list_C1 = Failed_index_list_C1_x;
elseif n_failed_C1 == length(Failed_index_list_C1_y)
    Failed_index_list_C1 = Failed_index_list_C1_y;
elseif n_failed_C1 == length(Failed_index_list_C1_z)
    Failed_index_list_C1 = Failed_index_list_C1_z;
end
if n_failed_C2 == length(Failed_index_list_C2_x)
    Failed_index_list_C2 = Failed_index_list_C2_x;
elseif n_failed_C2 == length(Failed_index_list_C2_y)
    Failed_index_list_C2 = Failed_index_list_C2_y;
elseif n_failed_C2 == length(Failed_index_list_C2_z)
    Failed_index_list_C2 = Failed_index_list_C2_z;
end
if n_failed_C5 == length(Failed_index_list_C5_x)
    Failed_index_list_C5 = Failed_index_list_C5_x;
elseif n_failed_C5 == length(Failed_index_list_C5_y)
    Failed_index_list_C5 = Failed_index_list_C5_y;
elseif n_failed_C5 == length(Failed_index_list_C5_z)
    Failed_index_list_C5 = Failed_index_list_C5_z;
end
%initialize an array that will only contain the values of failed simulations
MEKF_failed_average_magnitude_angle_error = zeros(3,n_failed_C1); 
MEKF_failed_average_magnitude_angular_velocities_error = zeros(3,n_failed_C2); 
MEKF_failed_average_mangitude_angle_estimation_error = zeros(3,n_failed_C5);

for i=1:n_failed_C1
    failed_index = Failed_index_list_C1(i);
    MEKF_failed_average_magnitude_angle_error(:,i) = MEKF_average_magnitude_angle_error(failed_index,:);
end

for i=1:n_failed_C2
    failed_index = Failed_index_list_C2(i);
    MEKF_failed_average_magnitude_angular_velocities_error(:,i) = MEKF_average_magnitude_angular_velocities_error(failed_index,:);
end

for i=1:n_failed_C5
    failed_index = Failed_index_list_C5(i);
    MEKF_failed_average_mangitude_angle_estimation_error(:,i) = Attitude_average_mangnitude_real_vs_estimated(failed_index,:);
end


%Plot the average magnitude of the angles
figure;
tiledlayout(3,1);
nexttile;
%plot the average magnitude of the angle error
hold on;
scatter(Failed_index_list_C1,MEKF_failed_average_magnitude_angle_error(1,:));
scatter(Failed_index_list_C1,MEKF_failed_average_magnitude_angle_error(2,:));
scatter(Failed_index_list_C1,MEKF_failed_average_magnitude_angle_error(3,:));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angle error (in °)');
title('Average magnitude of the angle error');
%plot the average magnitude of the angular velocities error
nexttile;
hold on;
scatter(Failed_index_list_C2,MEKF_failed_average_magnitude_angular_velocities_error(1,:));
scatter(Failed_index_list_C2,MEKF_failed_average_magnitude_angular_velocities_error(2,:));
scatter(Failed_index_list_C2,MEKF_failed_average_magnitude_angular_velocities_error(3,:));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angular velocities error (in °/s)');
title('Average magnitude of the angular velocities error');
%plot the average magnitude of the angle estimation error
nexttile;
hold on;
scatter(Failed_index_list_C5,MEKF_failed_average_mangitude_angle_estimation_error(1,:));
scatter(Failed_index_list_C5,MEKF_failed_average_mangitude_angle_estimation_error(2,:));
scatter(Failed_index_list_C5,MEKF_failed_average_mangitude_angle_estimation_error(3,:));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angular velocities error (in °/s)');
title('Average magnitude of the angle estimation error');