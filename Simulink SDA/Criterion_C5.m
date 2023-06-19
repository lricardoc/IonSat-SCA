
n = length(MEKF_angle_estimation_error_x(1,:)); %number of simulations
t_sim = length(MEKF_angle_estimation_error_x(:,1)); %duration of the simulations
t_steady_state = round(t_sim*0.85);

%% Criteria 5 : on the angle estimator error  
% The uncertainty on the pointing accuracy must be between -3° and +3°
%In other word
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
        if (mean(MEKF_angle_estimation_error_x(j,i))>3) || (mean(MEKF_angle_estimation_error_x(j,i))<-3)
            C5_failed_x = C5_failed_x+1;
        end
        if (mean(MEKF_angle_estimation_error_y(j,i))>3) || (mean(MEKF_angle_estimation_error_y(j,i))<-3)
            C5_failed_y = C5_failed_y+1;
        end
        if (mean(MEKF_angle_estimation_error_z(j,i))>3) || (mean(MEKF_angle_estimation_error_z(j,i))<-3)
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

%% Criteria 5bis : on the angle error rate, in caseboth PID and PID fails C5 at 100%
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
        if (mean(MEKF_angle_estimation_error_x(j,i))>9) || (mean(MEKF_angle_estimation_error_x(j,i))<-9)
            C5bis_failed_x = C5bis_failed_x+1;
        end
        if (mean(MEKF_angle_estimation_error_y(j,i))>9) || (mean(MEKF_angle_estimation_error_y(j,i))<-9)
            C5bis_failed_y = C5bis_failed_y+1;
        end
        if (mean(MEKF_angle_estimation_error_z(j,i))>9) || (mean(MEKF_angle_estimation_error_z(j,i))<-9)
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
%Get the max number of failed simulations
n_failed_C5_x = length(Failed_index_list_C5_x);
n_failed_C5_y = length(Failed_index_list_C5_y);
n_failed_C5_z = length(Failed_index_list_C5_z);
n_failed_C5 = max([n_failed_C5_x;n_failed_C5_y;n_failed_C5_z]); % max number of C5 failed simlations


%Get the index of the failed simulations
Failed_index_list_C5 = []; 

if n_failed_C5 == length(Failed_index_list_C5_x)
    Failed_index_list_C5 = Failed_index_list_C5_x;
elseif n_failed_C5 == length(Failed_index_list_C5_y)
    Failed_index_list_C5 = Failed_index_list_C5_y;
elseif n_failed_C5 == length(Failed_index_list_C5_z)
    Failed_index_list_C5 = Failed_index_list_C5_z;
end


%initialize an array that will only contain the values of failed simulations
MEKF_failed_average_magnitude_angle_estimation_error = zeros(3,n_failed_C5); 

for i=1:n_failed_C5
    failed_index = Failed_index_list_C5(i);
    MEKF_failed_average_magnitude_angle_estimation_error(:,i) = MEKF_failed_average_magnitude_angle_estimation_error(failed_index,:);
end



%plot the average magnitude of the angle error
%hold on;
scatter(Failed_index_list_C5,PID_failed_average_magnitude_angle_error(1,:));
scatter(Failed_index_list_C5,PID_failed_average_magnitude_angle_error(2,:));
scatter(Failed_index_list_C5,PID_failed_average_magnitude_angle_error(3,:));
legend('X axis','Y axis','Z axis')
xlabel('n-th simulation');
ylabel('Angle estimation error (in °)');
title('Average magnitude of the angle estimation error (real vs estimated attitude)');

