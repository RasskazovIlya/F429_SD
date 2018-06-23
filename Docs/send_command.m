clear all;
delete(instrfindall);
s1 = serial('COM4','DataBits',8); 
s1.InputBufferSize = 2000; 
s1.BaudRate = 230400; 
s1.Timeout = 10; 

fopen(s1);

command = 'f';
fwrite(s1,command);
pause(0.001);
str8bit = fread(s1, 4);

fclose(s1);