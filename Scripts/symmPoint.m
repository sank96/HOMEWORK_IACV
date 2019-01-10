function [p] = symmPoint(point,toFit,vpY)
%SYMMPOINT Summary of this function goes here
%   Detailed explanation goes here

[xx, yy] = fillLine(vpY, point, 1000);
[d, x, y] = nearest(toFit(1), toFit(2), xx, yy);
p = [x;y;1];
end

