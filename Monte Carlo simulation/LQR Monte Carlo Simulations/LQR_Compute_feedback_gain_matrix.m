%% Compute the feedback gain of the LQR controller
% sat_inertia = [0.07020  0.001720  -0.002260;
%                 0.001720  0.1130  -0.0003440;
%                -0.002260  -0.0003440  0.1600];
B = [zeros(3);
     sat_inertia^-1];
q = 0.004;
Q = [q  0  0  0  0  0;
     0  q  0  0  0  0;
     0  0  q  0  0  0;
     0  0  0  q  0  0;
     0  0  0  0  q  0;
     0  0  0  0  0  q];
r=4000;
R = [r  0  0;
     0  r  0;
     0  0  r];


Kq1 = [];
Kq2 = [];
Kq3 = [];
Kw1 = [];
Kw2 = [];
Kw3 = [];
parfor i = -6000:6000 %range of the reaction wheel speed/ "parfor" is the same as a "for" but faster
    RW_speed_matrix = [i;
                    i;
                    i;
                    0];
    h_w = sat.wheel.repartition_matrix_4RW*sat.wheel.inertia*RW_speed_matrix; %angular momentum of the reaction wheel
    Skew_h_w = [0  -h_w(3)  h_w(2);
                h_w(3)  0  -h_w(1);
                -h_w(2)  h_w(1)  0];
    A = [zeros(3) 0.5*eye(3);  % state matrix A
        zeros(3) Skew_h_w];
    K = lqr(A,B,Q,R);  %Feedback gain matrix
    Kq1 = [Kq1;K(1)];
    Kq2 = [Kq2;K(5)];
    Kq3 = [Kq3;K(9)];
    Kw1 = [Kw1;K(10)];
    Kw2 = [Kw2;K(14)];
    Kw3 = [Kw3;K(18)];
end
% mean(Kq1),mean(Kq2),mean(Kq1),mean(Kw1),mean(Kw2),mean(Kw3)
K = [mean(Kq1)  0  0  mean(Kw1)  0  0;
    0  mean(Kq2)  0  0  mean(Kw2)  0;
    0  0  mean(Kq3)  0  0  mean(Kw3)]