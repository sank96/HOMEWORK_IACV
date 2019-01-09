function [pm] = crossRatioMid(p1,p2,inf)
%CROSSRATIO Summary of this function goes here
%   Detailed explanation goes here

% direction
d = -abs(p1-inf)/norm((p1-inf));
% chose vanishing point as zero
x2 = 0;
% distance from zero to y and z
y =  pdist([inf(1) inf(2); p1(1) p1(2)],'euclidean');
z =  pdist([inf(1) inf(2); p2(1) p2(2)],'euclidean');

% distance from zero to x1
x1 = (y*x2 + z*x2 + 2*y*z) /...
    (2*x2 - z - y);

% find the original position on the line
pm = inf + x1*d;
end

