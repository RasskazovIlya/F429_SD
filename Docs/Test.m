delete(instrfindall);
s1 = serial('COM9','DataBits',8); 
s1.InputBufferSize = 2000; 
s1.BaudRate = 230400; 
s1.Timeout = 5; 

fopen(s1);

command = 'f';
fwrite(s1,command);

num_str8bit = fread(s1, 4);
num_str = bitor( bitor( bitshift(num_str8bit(4),24), bitshift(num_str8bit(3),16)), bitor( bitshift(num_str8bit(2),8), num_str8bit(1) ) );

temp = zeros(16*num_str,1);
temp_accel = zeros(6, 1);
temp_gyro = zeros(6, 1);
time_mark8bit = zeros(4, 1);

for i = 0:num_str
    temp_accel = fread(s1, 6);
    temp_gyro = fread(s1, 6);
    time_mark8bit = fread(s1, 4);
    
    for j = 1:6
        temp(16*i+j) = temp_accel(j);
        temp(16*i+j+6) = temp_gyro(j);
    end
    for j = 1:4
        temp(16*i+j+10) = time_mark8bit(j);
    end
%    temp(7*i) = bitor( bitor( bitshift(time_mark8bit(4),24), bitshift(time_mark8bit(3),16)), bitor( bitshift(time_mark8bit(2),8), time_mark8bit(1) ) );
    
end

fclose(s1);
