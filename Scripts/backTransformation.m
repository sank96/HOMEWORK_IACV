function [Hr] = backTransformation(vp1,vp2,w)
%BACKTRANSFORMATION It returns the matrix of image rectification
%   Given two wanishing point and w it return the matrix of back
%   rectification

% line at infininty
lineInf = cross(vp1, vp2);
lineInf = lineInf/lineInf(3);

syms a b;
points = [a ; b ; 1];

% two equations
A1 = points.' * lineInf;
A2 = (points.')* w *points;

% find the solutions
sol = solve([A1 A2], [a b]);
x = double(sol.a).';
y = double(sol.b).';

% circular points
I = [x(1) ; y(1) ; 1];
J = [x(2) ; y(2) ; 1];

Cinf = I*J' + J*I';
[U,S,V] = svd(Cinf);          % A = U*S*V'

s11 = S(1,1);
s22 = S(2,2);
T = [ sqrt(s11)     0       0 ;...
        0       sqrt(s22)   0 ;...
        0           0       1];

bm = U*T;
Hr=inv(bm);
Hr=Hr/Hr(3,3);

end

