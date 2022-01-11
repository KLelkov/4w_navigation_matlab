function state_prime = ukfNavPredict(state, sensors)
    
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
    
    
    
    %% Unscented transform
    dim = 9;
    n = 2 * dim + 1; % number of sigma-points
    k = 15;
    a = 0.25;
    l = a * a * (dim + k) - dim;
    b = 2; % for gaussians
    Xs = zeros(dim, n);
    Ws = zeros(2, n);
    Xprime = zeros(dim, 1);
    Pprime = zeros(dim, dim);
    
    % The first column repeats the state vector
    Xs(:,1) = state.X; % mean
    Shift = chol((dim+l)*state.P);
    % For each parameter in X we need to calculate 18 additional
    % sigma-points. This can be done by shifting the mean by values in the
    % Shift matrix (that correlates with covariance matrix P)
    for i = 1:dim
        for j = 1:dim
            Xs(i, j+1) = Xs(i,1) + Shift(i, j);
            Xs(i, j+1 + dim) = Xs(i,1) - Shift(i, j);
        end
    end
    % Now we need to compute weights for each sigma point in Xs.
    % For each column in Xs we need mean_weight and covariance_weight
    % The weights for the first column are the largest
    Ws(1,1) = 1 / (dim + l);
    Ws(2,1) = Ws(1,1) + (1 - a*a + b);
    for i = 2:n
        Ws(1,i) = 1 / (2*dim + 2*l);
        Ws(2,i) = Ws(1,i);
    end
    % Normalize the weights
    sumW1 = sum(Ws(1,:));
    sumW2 = sum(Ws(2,:));
    Ws(1,:) = Ws(1,:) ./ sumW1;
    Ws(2,:) = Ws(2,:) ./ sumW2;
    
    %% Prediction
    % Motion transform (apply system dynamic F)

    % Степень доверия предсказаниям (чем меньше коэф. - тем больше ему
    % доверяем)
    
    
    % Apply prediction update to every column of Xs
    for j = 1:n
        Xs(1,j) = Xs(1,j) + Xs(4,j)*cos(Xs(3,j))*dt; % X
        Xs(2,j) = Xs(2,j) + Xs(4,j)*sin(Xs(3,j))*dt; % Y
        Xs(3,j) = Xs(3,j) + Xs(5,j)*dt; % Heading
        Xs(4,j) = rw / 4.0 * ( Xs(6,j) + Xs(7,j) + Xs(8,j) + Xs(9,j) ); % V
        Xs(5,j) = (tangf-tangr)/(lf+lr) * Xs(4,j); % dHeading
%         Xs(5,j) = rw / (2 * lw) * (Xs(6,j) - Xs(7,j) + Xs(8,j) - Xs(9,j));
        Xs(6,j) = Xs(6,j); % w1
        Xs(7,j) = Xs(7,j); % w2
        Xs(8,j) = Xs(8,j); % w3
        Xs(9,j) = Xs(9,j); % w4
    end
    
    %% Reverse unscented transform
    % Derive Xprime as weighted mean
    for i = 1:dim
        Xprime(i) = 0;
        for j = 1:n
            Xprime(i) = Xprime(i) + Xs(i,j)*Ws(1,j);
        end
    end
    % Derive Pprime as weighted covariance
    for j = 1:n
%         disp(j)
        a = Ws(2,j) * ((Xs(:,j) - Xprime) * (Xs(:,j) - Xprime)');
%         disp(a)
        Pprime = Pprime + a;
    end
    Pprime = Pprime + diag(U);
   
    Xprime(3) = wrapToPi(Xprime(3));
    state_prime = state;
    state_prime.P = Pprime;
    state_prime.X = Xprime;
    
end

