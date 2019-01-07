function [pm] = crossRatioMid(p1,p2,inf)
%CROSSRATIO Summary of this function goes here
%   Detailed explanation goes here

d = -abs(p2-p1)/norm((p2-p1));
x2 = 0;
y =  pdist([inf(1) inf(2); p1(1) p1(2)],'euclidean');
z =  pdist([inf(1) inf(2); p2(1) p2(2)],'euclidean');

x1 = (y*x2 + z*x2 + 2*y*z) /...
    (2*x2 - z - y);

pm = inf + x1*d;
end

