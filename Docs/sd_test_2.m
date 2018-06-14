delete(instrfindall);
s1 = serial('COM4','DataBits',8); 
s1.InputBufferSize = 2000; 
s1.BaudRate = 230400; 
s1.Timeout = 10; 

Max_Write_Size = 249;

for j = 1:5
    fopen(s1);

    temp_accel1 = fread(s1, 2*Max_Write_Size);
    accel1 = zeros(Max_Write_Size,1);
    
    temp_gyro1 = fread(s1, 2*Max_Write_Size);
    gyro1 = zeros(Max_Write_Size,1);

    temp_accel2 = fread(s1, 2*Max_Write_Size);
    accel2 = zeros(Max_Write_Size,1);

    temp_gyro2 = fread(s1, 2*Max_Write_Size);
    gyro2 = zeros(Max_Write_Size,1);

    for i = 1:Max_Write_Size
        accel1(i) = bitor( bitshift(temp_accel1(2*i),8), temp_accel1(2*i-1) );
    end
    
%         for i = 1:Max_Write_Size
%             accel2(i) = bitor( bitshift(temp_accel2(2*i),8), temp_accel2(2*i-1) );
%         end

    for i = 1:Max_Write_Size
        gyro1(i) = bitor( bitshift(temp_gyro1(2*i),8), temp_gyro1(2*i-1) );
    end
% 
%         for i = 1:Max_Write_Size
%             gyro2(i) = bitor( bitshift(temp_gyro2(2*i),8), temp_gyro2(2*i-1) );
%         end

    accel1_1 = zeros(83,1);
    accel1_2 = zeros(83,1);
    accel1_3 = zeros(83,1);

    for i = 1:83
        accel1_1(i)=accel1(3*i-2);
        accel1_2(i)=accel1(3*i-1);
        accel1_3(i)=accel1(3*i);
    end

%         accel2_1 = zeros(Max_Write_Size/3,1);
%         accel2_2 = zeros(Max_Write_Size/3,1);
%         accel2_3 = zeros(Max_Write_Size/3,1);
% 
%         for i = 1:83
%             accel2_1(i)=accel2(3*i-2);
%             accel2_2(i)=accel2(3*i-1);
%             accel2_3(i)=accel2(3*i);
%         end

    gyro1_1 = zeros(83,1);
    gyro1_2 = zeros(83,1);
    gyro1_3 = zeros(83,1);

    for i = 1:83
        gyro1_1(i)=gyro1(3*i-2);
        gyro1_2(i)=gyro1(3*i-1);
        gyro1_3(i)=gyro1(3*i);
    end

%     gyro2_1 = zeros(Max_Write_Size/3,1);
%     gyro2_2 = zeros(Max_Write_Size/3,1);
%     gyro2_3 = zeros(Max_Write_Size/3,1);
% 
%     for i = 1:83
%         gyro2_1(i)=gyro2(3*i-2);
%         gyro2_2(i)=gyro2(3*i-1);
%         gyro2_3(i)=gyro2(3*i);
%     end

    subplot(6,1,1), plot(accel1_1);title('Accel x-axis');
    subplot(6,1,2), plot(accel1_2);title('Accel y-axis');
    subplot(6,1,3), plot(accel1_3);title('Accel z-axis');

    subplot(6,1,4), plot(gyro1_1);title('Gyro x-axis');
    subplot(6,1,5), plot(gyro1_2);title('Gyro y-axis');
    subplot(6,1,6), plot(gyro1_3);title('Gyro z-axis');
    drawnow;

    fclose(s1);
    
end
