%This script will write into a txt file the feedback gain coefficient according to the weight and state matrix (Q and R)
%%% WARNING THIS SCRIPT TAKES A OT OF COMPUTATION TIME %%%%

%% initialize constant parameters
%Update the inertia if it has changed
I_xx =  0.0702;
I_yy =  0.113;
I_zz =  0.16;
I_xy =  0.0017;
I_xz = -0.0023;
I_yz = -0.0003;
sat_inertia = [I_xx I_xy I_xz;
               I_xy I_yy I_yz;
               I_xz I_yz I_zz];

B = [zeros(3);
     sat_inertia^-1];
q = 0.004;
Q = [q  0  0  0  0  0;
     0  q  0  0  0  0;
     0  0  q  0  0  0;
     0  0  0  q  0  0;
     0  0  0  0  q  0;
     0  0  0  0  0  q];

%% initialize the text file 
output_filename = 'LQR_feedback_gain_updated_parameter_q0.004_r1000-10000.txt';  % create a txt file
fileID = fopen(output_filename,'w'); % open a txt file to write into
for r = 1000:1000:10000  % the range the the R parameter of the LQR controller 
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
        RW_speed = i;
        RW_speed_matrix = [RW_speed;
                        RW_speed;
                        RW_speed;
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
    fprintf(fileID,'%g, %d, %f, %f, %f, %f, %f, %f,\n',q,r,mean(Kq1),mean(Kq2),mean(Kq3),mean(Kw1),mean(Kw2),mean(Kw3));
end




