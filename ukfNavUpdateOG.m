function state_prime = ukfNavUpdateOG(state, sensors)
    
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
    

    %     X  Y     H.             V    dH. w1 w2 w3 w4
    H = [ 0, 0,     0,            0,     1, 0, 0, 0, 0; % dHeading_gyro
          0, 0,     0,            0,     0, 1, 0, 0, 0; % odo1
          0, 0,     0,            0,     0, 0, 1, 0, 0; % odo2
          0, 0,     0,            0,     0, 0, 0, 1, 0; % odo3
          0, 0,     0,            0,     0, 0, 0, 0, 1];% odo4

    Z = [sensors.gyro;
        sensors.w1;
        sensors.w2;
        sensors.w3;
        sensors.w4];
    % Дисперсии ошибок датчиков (паспортные данные)
    R = diag([sensors.gyro_error sensors.odo_error sensors.odo_error sensors.odo_error sensors.odo_error]);


    

    
    
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

