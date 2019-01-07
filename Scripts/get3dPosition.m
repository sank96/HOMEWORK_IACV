function [pxy, pyz, pxz] = get3dPosition(p,center, hxy, hyz, hxz)
%GET3DPOSITION Summary of this function goes here
%   Detailed explanation goes here

pxy = normalize(hxy*p);
pyz = normalize(hyz*p);
pxz = normalize(hxz*p);


cxy = normalize(hxy*center);
cyz = normalize(hyz*center);
cxz = normalize(hxz*center);

px = pdist([pxy(1:2) cxy(1:2)],'euclidean');
py = pdist([pxy(1:2) cxy(1:2)],'euclidean');
pz = pdist([pxy(1:2) cxy(1:2)],'euclidean');


end
