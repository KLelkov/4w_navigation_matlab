function [X, Y, Heading] = test_navigation(time, gamma1, gamma2, Gyro, Lat, Lon, Vn, Ve, w1, w2, w3, w4)

len = length(time);
len2 = floor(len/5);
    referenceNeeded = true;
    notMovedYet = true;
    validCoords = false;
    sensors.dt = 0.005;
    referenceLat = 0;
    referenceLon = 0;
    X = zeros(len2, 1);
    Y = zeros(len2, 1);
    Heading = zeros(len2, 1);
    Anr = zeros(len2, 1);
    time2 = zeros(len2, 1);
    
    
    % init sensors struct
    sensors.gps_error_pos = 4e-1;
    sensors.gps_error_vel = 8e-1;
    sensors.gyro_error = 1e-2;%8e-2;
    sensors.odo_error = 1e-0;
    sensors.gps_heading_error = 8e-2;
    
    %init Kalman state struct
    offset = 155 * pi/180;
    kalman_state.X = zeros(9,1);
%     kalman_state.X(3) = 260*pi/180; % set initial heading
    kalman_state.X(3) = offset; % set initial heading
    kalman_state.P = diag([100 100 10 10 10 10 10 10 10]);
    % errors:
    kalman_state.pos = 6e-2;
    kalman_state.h = 4e-2;
    kalman_state.v = 3e-3;
    kalman_state.dh = 1e-1;
    kalman_state.odo = 3e-2;
    
    cnt = 0;
    
    for i = 1:len
        if rem(i, 5) == 0
            cnt = cnt +1;
            time2(cnt) = time(i);
        if i > 5
            sensors.dt = (time(i) - time(i-5)) / 1000;
        end
        % EDIT upper limit!
        if w1(i) > 0.2 && w1(i) < 15
            sensors.w1 = w1(i);
        else
            sensors.w1= 0;
        end
        if w2(i) > 0.2 && w2(i) < 15
            sensors.w2 = w2(i);
        else
            sensors.w2= 0;
        end
        if w3(i) > 0.2 && w3(i) < 15
            sensors.w3 = w3(i);
        else
            sensors.w3= 0;
        end
        if w4(i) > 0.2 && w4(i) < 15
            sensors.w4 = w4(i);
        else
            sensors.w4= 0;
        end
        % EDIT radians!
        sensors.gammaf = gamma1(i) * pi / 180.0;
        sensors.gammar = gamma2(i) * pi / 180.0;
        if sensors.w1 ~= 0 || sensors.w2 ~= 0 || sensors.w3~= 0 || sensors.w4 ~= 0
            notMovedYet = false;
        end
        sensors.gyro = -Gyro(i);
        if sensors.w1 == 0 && sensors.w2 == 0 && sensors.w3 == 0 && sensors.w4 == 0
            sensors.gyro = 0;
        end
        validCoords = false;
        if Lat(i) > 53 && Lat(i) < 58
            if Lon(i) > 35 && Lon(i) < 40
                validCoords = true;
            end
        end
        if validCoords
            referenceNeeded = false;
            if referenceLat == 0 || referenceLon == 0
                referenceNeeded = true;
            end
            if referenceNeeded
                xproj = -0.5;
                yproj = 0;
                if cnt >1
                    xproj = -0.5 * cos(Heading(cnt-1)) - 0.00 * sin(Heading(cnt-1));
                    yproj = -0.5 * sin(Heading(cnt-1)) + 0.00 * cos(Heading(cnt-1));
                end
                shiftLat = 0; shiftLon = 0;
                if i > 1 && cnt > 1
                    shiftLat = X(cnt-1)  / 111111.1 - xproj / 111111.1;
                    shiftLon = (Y(cnt-1) - yproj)  / (111111.1 * cosd(Lat(i))) ;
                end
                referenceLat = Lat(i)  - shiftLat;
                referenceLon = Lon(i)  - shiftLon;
            end
            if notMovedYet
                xproj = -0.5;
                yproj = 0;
                if cnt >1
                    xproj = -0.5 * cos(Heading(cnt-1)) - 0.00 * sin(Heading(cnt-1));
                    yproj = -0.5 * sin(Heading(cnt-1)) + 0.00 * cos(Heading(cnt-1));
                end
                referenceLat = Lat(i)  - xproj / 111111.1;
                referenceLon = Lon(i) - yproj  / (111111.1 * cosd(Lat(i)));
            end
            sensors.gps_x = (Lat(i) - referenceLat) * 111111.1;
            sensors.gps_y = (Lon(i) - referenceLon) * (111111.1 * cosd(Lat(i)));
            sensors.gps_dx = Vn(i);
            sensors.gps_dy = Ve(i);
        end
        if rem(i, 100) == 0
            update.gps = 1;
        else
            update.gps = 0;
        end
        
        update.gyro = 1;
        % DEBUG
%         update.gps = 0;
%         update.gyro = 0;
        
        kalman_state = ukfNav(kalman_state, sensors, update);
        X(cnt) = kalman_state.X(1);
        Y(cnt) = kalman_state.X(2);
        Heading(cnt) = wrapToPi(kalman_state.X(3));
        Anr(cnt) = kalman_state.X(5);
        end
    end
    close all
    
    [North, East, Head, Rot, veln, vele] = gps2meters(Lat, Lon);
    
    figure
    plot(Y, X, 'b', 'LineWidth', 2);
    axis equal;
    grid on;
    hold on;
    plot(East, North, 'k', 'LineWidth', 0.5);
    xlabel('Y_g (East), m')
    ylabel('X_g (North), m')
    legend UKF GPS
    
    
    figure;
    plot(time2, Heading, 'b')
    hold on;
    grid on
%     plot(time2, Head, 'k');
    legend filter raw
    title 'Heading'
    
    figure;
    plot(time2, Anr, 'b')
    hold on;
    grid on
%     plot(time2, Rot, 'k');
%     plot(time2, -Gyro, 'r');
    legend filter raw gyro
    
%     figure;
%     plot(time2, Anr, 'b')
%     hold on;
%     grid on
%     plot(time2, gamma1*pi/180, 'r');
%     plot(time2, gamma2*pi/180, 'g');
%     plot(time2, -Gyro, 'k');
%     legend filter_anr gamma1 gamma2 gyro
    
%     figure('Name', 'Localization Error');
%     Err = zeros(len, 1);
%     for i = 1 : len
%         Ey = Y(i) - East(i);
%         Ex = X(i) - North(i);
%         Err(i) = sqrt(Ey^2 + Ex^2);
%     end
    
%     plot(time2, Err, 'b', 'LineWidth', 2)
%     grid on
    
    
    [Xloc, Yloc] = ned2local(X, Y, offset);
    [Nloc, Eloc] = ned2local(North, East, offset);
    figure('Name', 'Trajectory in local frame')
    plot(Yloc, Xloc, 'b', 'LineWidth', 2);
    axis equal;
    grid on;
    hold on;
    plot(Eloc, Nloc, 'k', 'LineWidth', 0.5);
    xlabel('Y_g (East), m')
    ylabel('X_g (North), m')
    legend UKF GPS
    
%     figure('Name', 'Heading Error');
%     Herr = wrapToPi(Heading - Head);
%     plot(time, Herr*180/pi, 'b', 'LineWidth', 2)
%     grid on
%     ylim([-90 90])
    
%     figure('Name', 'gps velocity');
%     plot(time, Vn, 'b')
%     grid on
%     hold on;
%     plot(time, Ve, 'r')
%     plot(time, veln, '--b')
%     plot(time, vele, '--r')
end