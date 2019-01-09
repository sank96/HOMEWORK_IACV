function [P] = findP(p1,p2,p3,p4, P1,P2,P3,P4)
%FINDP Summary of this function goes here
%   Detailed explanation goes here

syms p11 p12 p13 p14 p21 p22 p23 p24 p31 p32 p33 p34;

Ps = [p11, p12, p13, p14;...
        p21, p22, p23, p24;...
        p31, p32, p33, p34]
eq1 = Ps*P1 == p1
eq2 = Ps*P2 == p2;
eq3 = Ps*P3 == p3;
eq4 = Ps*P4 == p4;

sol = solve([eq1 eq2 eq3 eq4], [p11 p12 p13 p14 p21 p22 p23 p24 p31 p32 p34]);

P = [   sol.p11     sol.p12     sol.p13     sol.p14;...
        sol.p21     sol.p22     sol.p23     sol.p24;...
        sol.p31     sol.p32     sol.p33     sol.p34]
end

