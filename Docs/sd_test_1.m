s1 = serial('COM4','DataBits',8); 
s1.InputBufferSize = 500; 
s1.BaudRate = 115200; 
s1.Timeout = 10; 

fopen(s1);

temp_accel1 = fread(s1, 198);
accel1 = zeros(99,1);
kek1 = dec2hex(temp_accel1);

for i = 1:99
    accel1(i) = bitor( bitshift(temp_accel1(2*i),8), temp_accel1(2*i-1) );
end
kek2 = dec2hex(accel1);

temp_accel2 = fread(s1, 198);
accel2 = zeros(99,1);
kek3 = dec2hex(temp_accel2);

for i = 1:99
    accel2(i) = bitor( bitshift(temp_accel2(2*i),8), temp_accel2(2*i-1) );
end
kek4 = dec2hex(accel2);

accel1_1 = zeros(33,1);
accel1_2 = zeros(33,1);
accel1_3 = zeros(33,1);

for i = 1:33
    accel1_1(i)=accel1(3*i-2);
    accel1_2(i)=accel1(3*i-1);
    accel1_3(i)=accel1(3*i);
end

accel2_1 = zeros(33,1);
accel2_2 = zeros(33,1);
accel2_3 = zeros(33,1);

for i = 1:33
    accel2_1(i)=accel2(3*i-2);
    accel2_2(i)=accel2(3*i-1);
    accel2_3(i)=accel2(3*i);
end

subplot(6,1,1), plot(accel1_1);title('Accelerometer x-axis');
subplot(6,1,2), plot(accel1_2);title('Acceleromete y-axis');
subplot(6,1,3), plot(accel1_3);title('Acceleromete z-axis');

subplot(6,1,4), plot(gyro1_1);title('File x-axis');
subplot(6,1,5), plot(gyro1_2);title('File y-axis');
subplot(6,1,6), plot(gyro1_3);title('File z-axis');

fclose(s1);
