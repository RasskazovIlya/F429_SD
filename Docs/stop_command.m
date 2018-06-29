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

command = 'h';
fwrite(s1,command);%send command to stop MPU

fclose(s1);