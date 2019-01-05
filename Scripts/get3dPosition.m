function [px, py, pz] = get3dPosition(p,center, hxy, hyz, hxz)
%GET3DPOSITION Summary of this function goes here
%   Detailed explanation goes here

pxy = hxy*p;
pxy = pxy/pxy(3);
pyz = hyz*p;
pyz = pyz/pyz(3);
pxz = hxz*p;
pxz = pxz/pxz(3);

cxy = hxy*center;
cxy = cxy/cxy(3);
cyz = hyz*center;
cyz = cyz/cyz(3);
cxz = hxz*center;
cxz = cxz/cxz(3);


px = pdist([pxy(1:2) cxy(1:2)],'euclidean')
py = pdist([pxy(1:2) cxy(1:2)],'euclidean')
pz = pdist([pxy(1:2) cxy(1:2)],'euclidean')


end
