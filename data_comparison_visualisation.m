
focus_on = "x"; %variable to focus on

timestep = 1;




%% Process data

full_model = readtable('B_ECI_from_full_IGRF_model.txt');
dipole_model = readtable('B_ECI_from_dipole_model_nmax-13.txt');

t_end = 7242; %Can change depending of the duration of a simulation
stop_time = t_end;

%retrieve data from B_ECI_from_dipole_model_nmax-x.txt file
B_full_1 = dipole_model.Var1(1:stop_time);
B_full_2 = dipole_model.Var2(1:stop_time);
B_full_3 = dipole_model.Var3(1:stop_time);

%retrieve date from  B_ECI_from_full_IGRF_model.txt file 
B_dipole_1 = full_model.Var1(1:stop_time);
B_dipole_2 = full_model.Var2(1:stop_time);
B_dipole_3 = full_model.Var3(1:stop_time);


time = [1:timestep:t_end]';

%Compute the difference in percentage
Gap_1 = zeros(1,t_end);
Gap_2 = zeros(1,t_end);
Gap_3 = zeros(1,t_end);

for i=1:t_end
    Gap_1(i) = B_full_1(i)/B_dipole_1(i);
    Gap_2(i) = B_full_2(i)/B_dipole_2(i);
    Gap_3(i) = B_full_3(i)/B_dipole_3(i);
end


%% Plotfocus_on

subplot(2,2,1:2)
plot(time,B_dipole_1,'Color','blue');
hold on
plot(time,B_dipole_2,'Color','red');
hold on
plot(time,B_dipole_3,'Color','green');
hold on
plot(time,B_full_1,'Color','blue','LineStyle','--');
hold on
plot(time,B_full_2,'Color','red','LineStyle','--');
hold on
plot(time,B_full_3,'Color','green','LineStyle','--');
hold on
legend("B-dipole_1","B-dipole_2","B-dipole_3","B-full_1","B-full_2","B-full_3");
title("Comparing the magnetic field B-ECI with the full model and B-ECI with dipole model nmax=13");
xlabel("time (s)");
grid on;

if (focus_on == "x")
    diff= double(B_dipole_1) - double(B_full_1);
    
    subplot(2,2,3)
    title(focus_on);
    plot(time,B_dipole_1,'Color','blue');
    hold on
    plot(time,B_full_1,'Color','red','LineStyle','--');
    hold on
    legend("dipole model","full model")
    title("focus on x axis");
    xlabel("time (s)");
    ylabel("magnetic field (T)");
    grid on;
    
    subplot(2,2,4)
    plot(time,diff,'Color','blue');
    legend("difference")
    title("difference");
    xlabel("time (s)");
    ylabel("magnetic field (T)");
    grid on;

end

if (focus_on == "y")
    diff= double(B_dipole_2) - double(B_full_2);
    
    subplot(2,2,3)
    plot(time,B_dipole_2,'Color','blue');
    hold on
    plot(time,B_full_2,'Color','red','LineStyle','--');
    hold on
    legend("dipole model","full model")
    title("focus on y axis");
    xlabel("time (s)");
    ylabel("magnetic field (T)");
    grid on;

    subplot(2,2,4)
    plot(time,diff,'Color','blue');
    legend("difference")
    title("difference");
    xlabel("time (s)");
    ylabel("magnetic field (T)");
    grid on;

    
end

if (focus_on == "z")
    diff= double(B_dipole_3) - double(B_full_3);
    
    subplot(2,2,3)
    title(focus_on);
    plot(time,B_dipole_3,'Color','blue');
    hold on
    plot(time,B_full_3,'Color','red','LineStyle','--');
    hold on
    legend("dipole model","full model")
    title("focus on z axis");
    xlabel("time (s)");
    ylabel("magnetic field (T)");
    grid on;

    subplot(2,2,4)
    plot(time,diff,'Color','blue');
    legend("difference")
    title("difference");
    xlabel("time (s)");
    ylabel("magnetic field (T)");
    grid on;

    
end

fprintf("mean error : %g", mean(abs(diff)));
fprintf(" for the %s", focus_on);
fprintf(" axis \n\r");


fprintf("x axis mean error in percentage : %g\n", mean(abs(Gap_1)));
fprintf("y axis mean error in percentage : %g\n", mean(abs(Gap_2)));
fprintf("z axis mean error in percentage : %g\n\r", mean(abs(Gap_3)));

fprintf("x axis max error in percentage : %g\n", max(abs(Gap_1)));
fprintf("y axis max error in percentage : %g\n", max(abs(Gap_2)));
fprintf("z axis max error in percentage : %g\n\r", max(abs(Gap_3)));


print(gcf,'comparo.png','-dpng','-r300');
    