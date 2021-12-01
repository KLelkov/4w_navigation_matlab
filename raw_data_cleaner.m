gps_starter = 1;
start_point_found = false;
nSim = length(GpsStatus);
i = 1;
while (~ start_point_found)
    if GpsStatus(i) >= 0
        gps_starter = i;
        start_point_found = true;
    end
    i = i + 1;
end
Lat = GpsLat(gps_starter:nSim);
Lon = GpsLon(gps_starter:nSim);
% TAKE CARE, GPS VELOCITY IS REVERSED
Vn = gpsVy(gps_starter:nSim);
Ve = gpsVx(gps_starter:nSim);
Gyro = LocWx(gps_starter:nSim);
Gamma1 = GammaF(gps_starter:nSim);
Gamma2 = GammaR(gps_starter:nSim);
Odo1 = LocOdoL(gps_starter:nSim);
Odo2 = LocOdoR(gps_starter:nSim);
Odo3 = LocOdo3(gps_starter:nSim);
Odo4 = LocOdo4(gps_starter:nSim);
Time = LocT(gps_starter:nSim)- LocT(gps_starter);
clearvars -except Lat Lon Vn Ve Gyro Gamma1 Gamma2 Odo1 Odo2 Odo3 Odo4 Time

figure('Name', 'Gps preview');
plot(Lon, Lat, 'b', 'LineWidth', 2);
grid on;
xlabel('Lon, deg')
ylabel('Lat, deg')

figure('Name', 'Odometry preview');
plot(Time ./1000, Odo1, 'b', 'LineWidth', 1);
grid on; hold on;
plot(Time ./1000, Odo2, 'r', 'LineWidth', 1);
plot(Time ./1000, Odo3, 'g', 'LineWidth', 1);
plot(Time ./1000, Odo4, 'c', 'LineWidth', 1);
xlabel('Time, sec')
ylabel('Wheel rotation speed, rad/sec')