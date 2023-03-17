
output_filename = 'Aero_torque_semimarjorsaxis_6678_mode_4.txt';

fileID = fopen(output_filename,'w');

for i=2:length(out.aerotorque.Data)
    fprintf(fileID,'%d, %d, %d,\n',out.aerotorque.Data(i,:));
end

fclose(fileID);