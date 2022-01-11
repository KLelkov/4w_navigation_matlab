% [X, Y, Heading] = test_navigation(Time, Gamma1, Gamma2, Gyro, Lat, Lon, Vn, Ve, Odo1, Odo2, Odo3, Odo4);
[X, Y, Heading] = python_ukf(Time, Gamma1, Gamma2, Gyro, Lat, Lon, Vn, Ve, Odo1, Odo2, Odo3, Odo4, gpsStatus);
close all
figure
plot (Y, X, 'b')
hold on
grid on
plot (navY, navX, 'r')