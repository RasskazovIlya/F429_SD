clear all;
delete(instrfindall);
s1 = serial('COM23','DataBits',8); 
s1.InputBufferSize = 2000; 
s1.BaudRate = 230400; 
s1.Timeout = 10; 

fopen(s1);

command = 's';
date = datestr(clock, 'dd_mm_yyyy_HH_MM_SS_FFF');
command = strcat(command, date);
fwrite(s1, command);

fclose(s1);