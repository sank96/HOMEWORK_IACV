function [ps] = getSymmetric(p,c,H)
%GETSYMMETRIC Summary of this function goes here
%   Detailed explanation goes here

pb = H*p;
pb = pb/pb(3);

cb = H*c;
cb = cb/cb(3);

psb = 2*cb - pb;
psb = psb/psb(3);

ps = inv(H)*psb;
ps = ps/ps(3);

end

