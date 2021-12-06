function [Xloc, Yloc] = ned2local(X, Y, offset)
    Xloc = zeros(length(X),1);
    Yloc = zeros(length(X),1);
    for i = 1:length(X)
        Xloc(i) = X(i) * cos(offset) + Y(i) * sin(offset);
        Yloc(i) = -X(i) * sin(offset) + Y(i) * cos(offset);
    end
end

