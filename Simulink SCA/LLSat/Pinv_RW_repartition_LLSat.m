%Original by X19 students
% alpha = pi / 6;
% sat.wheel.repartition_matrix_4RW = [[sin(alpha), sin(alpha), sin(alpha), sin(alpha)],
%     [cos(alpha), 0, -cos(alpha), 0],
%     [0, cos(alpha), 0, -cos(alpha)]];
% sat.wheel.repartition_metrix_4RW_inverse = pinv(sat.wheel.repartition_matrix_4RW);

%Up to date with CubeSpace RWA
sat.wheel.alpha = 26.6 * pi/180;      %from deg to rad
sat.wheel.repartition_matrix_4RW = [[cos(sat.wheel.alpha),      0,    -cos(sat.wheel.alpha),      0    ];...
                                    [     0,     cos(sat.wheel.alpha),     0,     -cos(sat.wheel.alpha)];...
                                    [sin(sat.wheel.alpha), sin(sat.wheel.alpha), sin(sat.wheel.alpha), sin(sat.wheel.alpha)]];
sat.wheel.repartition_matrix_4RW_inverse = pinv(sat.wheel.repartition_matrix_4RW);
sat.wheel.number = 6;
sat.wheel.repartition_matrix_6RW = [1, 0, 0, 1, 0, 0;...
                                    0, 1, 0, 0, 1, 0;...
                                    0, 0, 1, 0, 0, 1];
sat.wheel.repartition_matrix_6RW_inverse = pinv(sat.wheel.repartition_matrix_6RW);
sat.wheel.N = [0.5;0.5;0.5;0.5;0.5;0.5];