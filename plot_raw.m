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