function [North, East, Heading, Rotation, Vn, Ve] = gps2meters(lat, lon)
% Heading and Rotation speed approximation can be a little chunky, but it should give the general idea about how the robot was really moving
    len = length(lat);
    North = zeros(len, 1);
    East = zeros(len, 1);
    Vn = zeros(len, 1);
    Ve = zeros(len, 1);
    Heading = zeros(len, 1);
    Rotation = zeros(len, 1);
    for i = 1 : len
        East(i) = (lon(i) - lon(1)) * 111111 * cosd(lat(i));
        North(i) = (lat(i) - lat(1)) * 111111;
        if i > 1
            Vn(i) = (North(i) - North(i-1)) / 0.5;
            Ve(i) = (East(i) - East(i-1)) / 0.5;
            if Vn(i) ~= 0 || Ve(i) ~= 0
            Heading(i) = atan2(Ve(i), Vn(i));
            else
                Heading(i) = Heading(i-1);
            end
            Rotation(i) = (Heading(i) - Heading(i-1)) / 0.5;
            if Rotation(i) == 0
                Rotation(i) = Rotation(i-1);
            end
            if abs(Rotation(i)) >= 2
                Rotation(i) = Rotation(i-1);
            end
        end
    end

end

