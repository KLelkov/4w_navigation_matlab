function [X, Y, Heading] = python_ukf(time, gamma1, gamma2, Gyro, Lat, Lon, Vn, Ve, w1, w2, w3, w4, gpsStatus)
    len = length(time);
    divider = 5;
    X = zeros(floor(len/divider/2), 1);
    Y = zeros(floor(len/divider/2), 1);
    Heading = zeros(floor(len/divider/2), 1);
    cnt = 2;
    navMsgCount = 0;    
    heading_offset = 155 * pi / 180.0;
    posT = heading_offset;
    posX = 0;
    posY = 0;
    referenceLat = 0;
    referenceLon = 0;
    lastLat = 0;
    lastLon = 0;
    lastNvel = 0;
    lastEvel = 0;
    notMovedYet = true;
    lastTimestamp = 0;

    X(1) = posX;
    Y(1) = posY;
    Heading(1) = posT;
    
    % init sensors struct
    sensors.gps_error_pos = 4e-1;
    sensors.gps_error_vel = 8e-1;
    sensors.gyro_error = 1e-2;%8e-2;
    sensors.odo_error = 1e0;
    sensors.gps_heading_error = 8e-2;
    
    %init Kalman state struct
    kalman_state.X = zeros(9,1);
    kalman_state.X(3) = posT; % set initial heading
    kalman_state.P = diag([100 100 10 10 10 10 10 10 10]);
    % errors:
    kalman_state.pos = 6e-2;
    kalman_state.h = 4e-2;
    kalman_state.v = 3e-3;
    kalman_state.dh = 1e-1;
    kalman_state.odo = 3e-2;
    
    for i = 2:len
        
        % motors_callback ---
        if i == 2 || w1(i) ~= w1(i-1) || w2(i) ~= w2(i-1) || w3(i) ~= w3(i-1) || w4(i) ~= w4(i-1)
            if w1(i) > 0.2 && w1(i) < 15
                sensors.w1 = w1(i);
            else
                sensors.w1 = 0;
            end
            if w2(i) > 0.2 && w2(i) < 15
                sensors.w2 = w2(i);
            else
                sensors.w2 = 0;
            end
            if w3(i) > 0.2 && w3(i) < 15
                sensors.w3 = w3(i);
            else
                sensors.w3 = 0;
            end
            if w4(i) > 0.2 && w4(i) < 15
                sensors.w4 = w4(i);
            else
                sensors.w4 = 0;
            end
        end
        sensors.gammaf = gamma1(i) * pi / 180.0;
        sensors.gammar = gamma2(i) * pi / 180.0;
        % ---
        
        % gps_vel_callback ---
        if Vn(i) ~= Vn(i-1) || Ve(i) ~= Ve(i-1)
            if abs(Vn(i)) < 15 && abs(Ve(i)) < 15
                lastNvel = Vn(i);
                lastEvel = Ve(i);
            end
        end
        % ---
        
        % gps_pos_callback ---
        if Lat(i) ~= Lat(i-1) || Lon(i) ~= Lon(i-1)
            newLat = Lat(i);
            newLon = Lon(i);
            validCoords = false;
            if newLat > 53 && newLat < 58 && newLon > 35 && newLon < 40
                validCoords = true;
            end
            if validCoords && gpsStatus(i) >= 0
                referenceNeeded = false;
                if referenceLat == 0 || referenceLon == 0
                    referenceNeeded = true;
                end
                if referenceNeeded
                    xproj = -0.5 * cos(posT) - 0.0 * sin(posT);
                    yproj = -0.5 * sin(posT) + 0.0 * cos(posT);
                    shiftLat = (posX - xproj) / 111111.1;
                    shiftLon =  (posY - yproj) / (111111.1 * cosd(newLat));
                    referenceLat = newLat - shiftLat;
                    referenceLon = newLon - shiftLon;
                end
                if notMovedYet
                    xproj = -0.5 * cos(posT) - 0.0 * sin(posT);
                    yproj = -0.5 * sin(posT) + 0.0 * cos(posT);
                    referenceLat = newLat - xproj / 111111.1;
                    referenceLon = newLon - yproj / (111111.1 * cosd(newLat));
                end
            end
            lastLat = newLat;
            lastLon = newLon;           
        end
        % ---
        
        % locomotion_callback ---
        % dtCalc ---
        navMsgCount = navMsgCount + 1;
        if rem(navMsgCount, divider) == 0
            dt = 0;
            if lastTimestamp == 0
                dt = 0.005;
            else
                dt = (time(i) - lastTimestamp) / 1000;
            end
            lastTimestamp = time(i);
            % ---
            sensors.dt = dt;
            
            gyro = 0;
            update.gyro = true;
            if sensors.w1 ~= 0 || sensors.w2 ~= 0 || sensors.w3 ~= 0 || sensors.w4 ~= 0
                notMovedYet = false;
                gyro = -Gyro(i);
            end
            sensors.gyro = gyro;
            
            if lastLon ~= 0 && lastLat ~= 0
                update.gps = true;
                sensors.gps_x = (lastLat - referenceLat) * 111111.1;
                sensors.gps_y = (lastLon - referenceLon) * (111111.1 * cosd(lastLat));
                sensors.gps_dx = lastNvel;
                sensors.gps_dy = lastEvel;
            else
                update.gps = false;
            end
            
            % KALMAN PREDICT
            kalman_state = ukfNavPredict(kalman_state, sensors);
            
            if update.gps
                % KALMAN UPDATE OGS
               % kalman_state = ukfNavUpdateOGS(kalman_state, sensors);
            else
                % KALMAN UPDATE OG
                kalman_state = ukfNavUpdateOG(kalman_state, sensors);
            end
            
            % GET SOLUTION
            navX = kalman_state.X(1);
            navY = kalman_state.X(2);
            navT = kalman_state.X(3);
            
            if rem(navMsgCount, 10) == 0
                % PUBLISH
                X(cnt) = navX * cos(heading_offset) + navY * sin(heading_offset);
                Y(cnt) = -navX * sin(heading_offset) + navY * cos(heading_offset);
                Heading(cnt) = wrapToPi(navT - heading_offset);
                cnt = cnt + 1;
            end
            if update.gps
                update.gps = false;
                lastLat = 0;
                lastLon = 0;
                lastEvel = 0;
                lastNvel = 0;
            end
        end
        % ---
    end
    
end

