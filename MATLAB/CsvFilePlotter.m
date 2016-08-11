[num, txt, raw] = xlsread(...
    'imuData005POLL.csv');

time = num(1:end,1);
accelx = num(1:end,2);
accely = num(1:end,3);
accelz = num(1:end,4);
gyrox = num(1:end,5);
gyroy = num(1:end,6);
gyroz = num(1:end,7);

angleX = rad2deg(atan(accely...
    ./sqrt(accelx.^2+accelz.^2)));
angleY = rad2deg(atan(accelx...
    ./sqrt(accely.^2+accelz.^2)));

dt = 0.017;

compFangX = 0.98 * (angleX + gyrox * dt) + 0.02 * accelx;
compFangY = 0.98 * (angleY + gyroy * dt) + 0.02 * accely;

maggyro = sqrt(gyrox .^ 2 + gyroy .^ 2 + gyroz .^ 2);
magaccel = sqrt(accelx .^ 2 + accely .^ 2 + accelz .^ 2);

subplot(2,1,1)
plot(time(:),accelx(1:end))
hold on
plot(time(:),accely(1:end))
hold on
plot(time(:),accelz(1:end))
title('Acceleration')
xlabel('Seconds')
ylabel('Angle in Degrees of Pitch')

subplot(2,1,2)
plot(time(:),gyrox(1:end))
hold on
plot(time(:),gyroy(1:end))
hold on
plot(time(:),gyroz(1:end))
title('Angular Velocity')
xlabel('Seconds')
ylabel('Angle in Degrees of Roll')

%{
subplot(1,2,1)
plot(1:length(compFangX(1:end)),compFangX(1:end))
title('AngleX')
xlabel('Sample Number')
ylabel('Angle in Degrees of Pitch')

subplot(1,2,2)
plot(1:length(compFangY(1:end)),compFangY(1:end))
title('AngleY')
xlabel('Sample Number')
ylabel('Angle in Degrees of Roll')
%}
%{
subplot(2,2,3)
plot(1:length(magaccel(1:end)),magaccel(1:end))
title('MagAccel')
subplot(2,2,4)
plot(1:length(maggyro(1:end)),maggyro(1:end))
title('MagGyro')
%}
