[X, Y, Heading] = test_navigation(Time, Gamma1, Gamma2, Gyro, Lat, Lon, Vn, Ve, Odo1, Odo2, Odo3, Odo4);

hold on
plot (navY, navX, 'r')