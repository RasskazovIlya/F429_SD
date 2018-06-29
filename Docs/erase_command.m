delete(instrfindall);
s1 = serial('COM9','DataBits',8); 
s1.InputBufferSize = 2000; 
s1.BaudRate = 230400; 
s1.Timeout = 10; 

fopen(s1);

command = 'e';
fwrite(s1,command);

fclose(s1);