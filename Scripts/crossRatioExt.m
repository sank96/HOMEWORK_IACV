function [p2] = crossRatioExt(p1,pm,inf)
%CROSSRATIOEXT Return a symmetric point
%   Given a point p1, a midpoint pm and a vanishing point, it return a
%   point p2 such taht pm is the midpoint of the vector p1p2

% direction
d = (p1-pm)/norm((p1-pm));
x2 = 0;     % choose as zero
y =  pdist([inf(1) inf(2); p1(1) p1(2)],'euclidean');
x1 =  pdist([inf(1) inf(2); pm(1) pm(2)],'euclidean');

z = (2*x1*x2 - y*x2 - y*x1) /...
    (x2 + x1 -2*y);

p2 = inf + z*d;
end

