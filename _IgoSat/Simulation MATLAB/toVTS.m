%%%%%.........VTS file generation.....................%%%%%%

% Note:
% 1) Get the position & quaternion data from simulink
% 2) specify Vts directory
% 3) Run the code
% 4) select pos.txt & quat.txt in VTS interface  

ps = pos; % position from simulink
at=aty; %quaternion from simulink
tms = tme; % time from simulink


MatlabDir = 'C:\Users\IGOSAT\Desktop\IGOSat_ADCS2018_Simulation';
VtsDir = 'C:\Users\IGOSAT\Desktop\Antoine_IGOSatProject\VTS';
%MatlabDir = 'C:\Users\Navya\Google Drive\Internship-IGOSAT\SCAO\Navya krishna ACS 2016\trial 1';

% VtsDir = 'C:\Users\Navyakrishna\Downloads\Vts-WindowsNT-32bits-2.7';
% MatlabDir = 'C:\Users\Navyakrishna\Google Drive\Internship-IGOSAT\SCAO\Navya krishna ACS 2016\trial 1';

% Position generation
header1='CIC_OEM_VERS = 2.0';
header2='COMMENT Generated by Spacebel CCSDS library';
header3='CREATION_DATE = 2016-06-14T14:23:59.015921';
header4='ORIGINATOR = VTS Propagated Orbit';
header5='META_START';
header6='OBJECT_NAME = IGOSAT';
header7='OBJECT_ID = IGOSAT';
header8='CENTER_NAME = EARTH';
header9='REF_FRAME = EME2000';
header10='TIME_SYSTEM = UTC';
header11='META_STOP';

p = [58284*ones(size(ps,1),1) tms ps ];

% Quaternion generation
header21='CIC_AEM_VERS = 1.0';
header22='COMMENT Generated by Spacebel CCSDS library';
header23='CREATION_DATE = 2016-06-14T14:23:59.015921';
header24='ORIGINATOR = VTS Propagated Orbit';

header25='META_START';

header26='COMMENT Exemple de fichier de attitude';

header27='OBJECT_NAME = IGOSAT';
header28='OBJECT_ID = IGOSAT';

header29='REF_FRAME_A = EME2000';
header30='REF_FRAME_B = SC_BODY_1';
header31='ATTITUDE_DIR = A2B';

header32='TIME_SYSTEM = UTC';

header33='ATTITUDE_TYPE = QUATERNION';

header34='META_STOP';

q = [58284*ones(size(at,1),1) tms at ];

cd(VtsDir);

fid=fopen('pos.txt','w');
fprintf(fid, [ header1 '\n' header2 '\n' header3 '\n' header4 '\n\n' header5 '\n' header6 '\n' header7 '\n' header8 '\n' header9 '\n' header10 '\n\n' header11 '\n\n']);
fprintf(fid, '%5.0f %6.2f %f %f %f \n', [p]');
fclose(fid);

fid1=fopen('quat.txt','w');
fprintf(fid1, [ header21 '\n' header22 '\n' header23 '\n' header24 '\n\n' header25 '\n\n' header26 '\n\n' header27 '\n' header28 '\n\n' header29 '\n' header30 '\n' header31 '\n\n' header32 '\n\n' header33 '\n\n' header34 '\n\n']);
fprintf(fid1, '%5.0f %6.2f %f %f %f %f \n', [q]');
fclose(fid1);

cd(MatlabDir);