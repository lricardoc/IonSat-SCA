%CIC writer modified IonSat team X20 to use with Control_v6.slx (ACS)
disp('Opening out file');

data.pos = out.sat_pos.data/1000.0;
data.speed = out.sat_speed.data/1000.0;
%in km and km/s

data.quaternions = out.sat_Q.data;
%data.quaternions_ref = out.sat_Qref.data;
%data.quaternions_ref = out.sat_Qc.data;

%On récupère les données de temps
data.time = num2cell(datetime(out.T_JD.data,'convertfrom','juliandate','Format','yyyy-MM-dd''T''HH:mm:ss'));

%%METADATA CREATION

disp('Writing metadata');

metadata.Format = 'yyyy-MM-ddTHH:mm:ss';
metadata.creationTime = datestr(now,metadata.Format);
metadata.objectName = 'IONSAT';
metadata.objectID = 'IONSAT';
metadata.centerName = 'EARTH';
metadata.refFrame = 'ICRF';
metadata.refFrameA = 'ICRF';
metadata.refFrameB = 'SC_BODY_1';
metadata.timeSystem = 'UTC';
%metadata.startTime = datestr(data.time{1},metadata.Format);
%metadata.stopTime = datestr(data.time{end},metadata.Format);

%POS SPEED METADATA

fileID = fopen('./Data/data_pos&speed.txt','w');
fprintf(fileID,'CIC_OEM_VERS = 2.0\r\n');
fprintf(fileID,strcat('CREATION_DATE = ',metadata.creationTime,'\r\n'));
fprintf(fileID,'ORIGINATOR = IONSAT TEAM\r\n');
fprintf(fileID,'\r\n');
fprintf(fileID,'META_START\r\n');
fprintf(fileID,'\r\n');
fprintf(fileID,strcat('OBJECT_NAME = ',metadata.objectName,'\r\n'));
fprintf(fileID,strcat('OBJECT_ID = ',metadata.objectID,'\r\n'));
fprintf(fileID,strcat('CENTER_NAME = ',metadata.centerName,'\r\n'));
fprintf(fileID,strcat('REF_FRAME = ',metadata.refFrame,'\r\n'));
fprintf(fileID,strcat('TIME_SYSTEM = ',metadata.timeSystem,'\r\n'));
fprintf(fileID,'\r\n');
fprintf(fileID,'META_STOP\r\n');
fprintf(fileID,'\r\n');
fclose(fileID);

%ATTITUDE METADATA

fileID = fopen('./Data/data_quaternions.txt','w');
fprintf(fileID,'CIC_AEM_VERS = 1.0\r\n');
fprintf(fileID,strcat('CREATION_DATE = ',metadata.creationTime,'\r\n'));
fprintf(fileID,'ORIGINATOR = IONSAT TEAM\r\n');
fprintf(fileID,'\r\n');
fprintf(fileID,'META_START\r\n');
fprintf(fileID,'\r\n');
fprintf(fileID,strcat('OBJECT_NAME = ',metadata.objectName,'\r\n'));
fprintf(fileID,strcat('OBJECT_ID = ',metadata.objectID,'\r\n'));
fprintf(fileID,strcat('CENTER_NAME = ',metadata.centerName,'\r\n'));
fprintf(fileID,strcat('REF_FRAME_A =  ',metadata.refFrameA,'\r\n'));
fprintf(fileID,strcat('REF_FRAME_B =  ',metadata.refFrameB,'\r\n'));
fprintf(fileID,strcat('ATTITUDE_DIR = A2B\r\n'));
fprintf(fileID,strcat('TIME_SYSTEM = ',metadata.timeSystem,'\r\n'));
fprintf(fileID,strcat('ATTITUDE_TYPE = QUATERNION\r\n'));
fprintf(fileID,strcat('QUATERNION_TYPE = FIRST\r\n'));
fprintf(fileID,'\r\n');
fprintf(fileID,'META_STOP\r\n');
fprintf(fileID,'\r\n');
fclose(fileID);

%%METADATA END

% Some modifications
data.pos_s = squeeze(data.pos);
data.speed_s = squeeze(data.speed);
data.quaternions_s = squeeze(data.quaternions);
%
data.tableCIC_pos_speed = [data.time num2cell(data.pos_s') num2cell(data.speed_s)'];
data.table_CIC_quaternions = [data.time num2cell(data.quaternions_s')];
% data.table_CIC_quaternions_ref = [data.time num2cell(data.quaternions_ref)];
disp('Writing position and speed');
writecell(data.tableCIC_pos_speed,'./Data/tmp_pos&speed.txt','Delimiter',' ');
disp('Writing actual quaternion');
writecell(data.table_CIC_quaternions,'./Data/tmp_quaternions.txt','Delimiter','\t');
% disp('Writing command quaternion');
% writecell(data.table_CIC_quaternions_ref,'./Data/tmp_quaternions_ref.txt','Delimiter','\t');

%%ADD METADATA TO THE EXPORT FILES

disp('Adding data to metadata file');

% copyfile('./Data/data_quaternions.txt','./Data/data_quaternions_ref.txt');

fileID = fopen('./Data/data_pos&speed.txt','a');
fprintf(fileID, fileread('./Data/tmp_pos&speed.txt'));
fclose(fileID);
fileID = fopen('./Data/data_quaternions.txt','a');
fprintf(fileID, fileread('./Data/tmp_quaternions.txt'));
fclose(fileID);
% fileID = fopen('./Data/data_quaternions_ref.txt','a');
% fprintf(fileID, fileread('./Data/tmp_quaternions_ref.txt'));
% fclose(fileID);

disp('Done !');
% Specify the filename of the text file you want to delete
fileToDelete = './Data/tmp_pos&speed.txt';
% Use the delete function to remove the file
delete(fileToDelete);
delete('./Data/tmp_quaternions.txt');

% copyfile("./Data/data_pos&speed.txt","C:\CNES\Vts-WindowsNT-32bits-3.5.1\Data\IonSat") %The destination directory might be changed
% copyfile("./Data/data_quaternions.txt","C:\CNES\Vts-WindowsNT-32bits-3.5.1\Data\IonSat")
% copyfile("./Data/data_quaternions_ref.txt","C:\CNES\Vts-WindowsNT-32bits-3.5.1\Data\IonSat")
copyfile("./Data/data_pos&speed.txt","H:\My Drive\Matlab outputs\Data")
copyfile("./Data/data_quaternions.txt","H:\My Drive\Matlab outputs\Data")