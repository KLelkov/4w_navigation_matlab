function state_prime = ukfNavUpdateOGS(state, sensors)
    
    dt = sensors.dt;
    tangf = tan(sensors.gammaf);
    tangr = tan(sensors.gammar);
    lsx = -0.5;
    lsy = 0.0;
    lf = 0.4;
    lr = 0.45;
    lw = 0.625;
    rw = 0.254/2;
    U = [state.pos state.pos state.h state.v state.dh state.odo state.odo state.odo state.odo];
        
    %% Correction

    heading = state.X(3);
    xproj = lsx*cos(heading) - lsy*sin(heading);
    yproj = lsx*sin(heading) + lsy*cos(heading);
    

    %     X  Y  H.            V     dH. w1 w2 w3 w4
    H = [ 1, 0, 0,            0,     0, 0, 0, 0, 0; % Xs
          0, 1, 0,            0,     0, 0, 0, 0, 0; % Ys
          0, 0, 0, cos(heading), xproj, 0, 0, 0 ,0; % dXs
          0, 0, 0, sin(heading), yproj, 0, 0, 0, 0; % dYs
          0, 0, 0,            0,     1, 0, 0, 0, 0; % dHeading_gyro
          0, 0, 0,            0,     0, 1, 0, 0, 0; % odo1
          0, 0, 0,            0,     0, 0, 1, 0, 0; % odo2
          0, 0, 0,            0,     0, 0, 0, 1, 0; % odo3
          0, 0, 0,            0,     0, 0, 0, 0, 1; % odo4
          0, 0, 0,            0,     0, 0, 0, 0, 0];

    head_corr = 0;
    if sensors.gps_dx ~= 0
        head_corr = atan2(sensors.gps_dy, sensors.gps_dx);
    end
    Z = [sensors.gps_x - xproj;
        sensors.gps_y - yproj;
        sensors.gps_dx;
        sensors.gps_dy;
        sensors.gyro;
        sensors.w1;
        sensors.w2;
        sensors.w3;
        sensors.w4;
        head_corr];
    % Дисперсии ошибок датчиков (паспортные данные)
    R = diag([sensors.gps_error_pos sensors.gps_error_pos sensors.gps_error_vel sensors.gps_error_vel sensors.gyro_error sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error sensors.gps_heading_error]);
   
    
    if sqrt(sensors.gps_dx^2 + sensors.gps_dy^2) > 0.2 && ...
            sensors.w1 ~= 0 && sensors.w2 ~= 0 && sensors.w3 ~= 0 && sensors.w4 ~= 0
        H(10, 3) = 1;
    end
    
    
    y = Z - H*state.X; % невязка
    S = H * state.P * H' + R; 
    K = state.P * H' * inv(S);
    Xprime = state.X + K*y; % X'
    Pprime = (eye(9) - K*H)*state.P; % P'
    
    Xprime(3) = wrapToPi(Xprime(3));
    state_prime = state;
    state_prime.P = Pprime;
    state_prime.X = Xprime;
    
end

