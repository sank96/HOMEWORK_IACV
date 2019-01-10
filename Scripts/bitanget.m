function [lines] = bitanget(C1,C2)
%BITANGET Find the tangents to both conics input

syms a b;

% dual conic
C1star = inv(C1);
C2star = inv(C2);

% set of lines
l = [a; b; 1];
% systems equation
A1 = l.' * C1star * l;
A2 = l.' * C2star * l;

sol = solve([A1 A2], [a b]);

% convert symbolic values into variables with double precision
% https://it.mathworks.com/help/symbolic/double.html
x = double(sol.a).';
y = double(sol.b).';
onesV = ones(4, 1).';

% solutions, 4 lines
lines = [x; y; onesV];
end

