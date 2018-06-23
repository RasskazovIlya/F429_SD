clear all;
delete(instrfindall);
s1 = serial('COM23','DataBits',8); 
s1.InputBufferSize = 2000; 
s1.BaudRate = 230400; 
s1.Timeout = 10; 

fopen(s1);

command = 'h';
fwrite(s1,command);

fclose(s1);