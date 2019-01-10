function [p] = symmPoint(point,toFit,vpY)
%SYMMPOINT Given a potential symmetric point 
%   Return a possible symmetric point belong the line from point to
%   vanishing point

[xx, yy] = fillLine(vpY, point, 1000);
[d, x, y] = nearest(toFit(1), toFit(2), xx, yy);
p = [x;y;1];
end

