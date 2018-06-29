delete(instrfindall);
%<<<<<<< HEAD
%s1 = serial('COM23','DataBits',8); 
%=======
s1 = serial('COM9','DataBits',8); 
%>>>>>>> 8e05c99e73543878b9eee234f12655d4356e201c
s1.InputBufferSize = 2000; 
s1.BaudRate = 230400; 
s1.Timeout = 10; 

fopen(s1);

command = 's';
date = datestr(clock, 'dd_mm_yyyy_HH_MM_SS_FFF');%get date in string form
command = strcat(command, date);%command format: 'sdd_mm_yyyy_HH_MM_SS_FFF'
fwrite(s1, command);%send command to start MPU

fclose(s1);