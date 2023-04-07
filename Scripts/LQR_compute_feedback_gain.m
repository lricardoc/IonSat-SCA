%%% WARNING THIS SCRIPT TAKES A OT OF COMPUTATION TIME %%%%

%% initialize constant parameters
B = [zeros(3);
     sat.inertia^-1];
q = 0.004;
%r = 100;

Q = [q  0  0  0  0  0;
     0  q  0  0  0  0;
     0  0  q  0  0  0;
     0  0  0  q  0  0;
     0  0  0  0  q  0;
     0  0  0  0  0  q];



%%initialize the text file 
output_filename = 'LQR_feedback_gain_bis.txt';

fileID = fopen(output_filename,'w');
for r = 100:100:10000
    
    R = [r  0  0;
         0  r  0;
         0  0  r];    
    
    Kq1 = [];
    Kq2 = [];
    Kq3 = [];
    Kw1 = [];
    Kw2 = [];
    Kw3 = [];
    
    parfor i = -6000:6000
        RW_speed = i;
        RW_speed_matrix = [RW_speed;
                        RW_speed;
                        RW_speed;
                        0];
    
        h_w = sat.wheel.repartition_matrix_4RW*sat.wheel.inertia*RW_speed_matrix;
    
        Skew_h_w = [0  -h_w(3)  h_w(2);
                    h_w(3)  0  -h_w(1);
                    -h_w(2)  h_w(1)  0];
    
        A = [zeros(3) 0.5*eye(3);
            zeros(3) Skew_h_w];
        
        K = lqr(A,B,Q,R);

        Kq1 = [Kq1;K(1)];
        Kq2 = [Kq2;K(5)];
        Kq3 = [Kq3;K(9)];
        Kw1 = [Kw1;K(10)];
        Kw2 = [Kw2;K(14)];
        Kw3 = [Kw3;K(18)];

    
    end
    fprintf(fileID,'%g, %d, %f, %f, %f, %f, %f, %f,\n',q,r,mean(Kq1),mean(Kq2),mean(Kq1),mean(Kw1),mean(Kw2),mean(Kw3));
end




