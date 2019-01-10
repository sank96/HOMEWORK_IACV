function [P1, P2, M, m] = plotSyms(p1,p2,center, HrXY, HrYZ, position2d, position3d, vpY)
%PLOTSYMS Summary of this function goes here
%   Detailed explanation goes here

p2r = symmPoint(p1, p2, vpY);
% midpoint
pm = crossRatioMid(p1, p2r, vpY);
m = pm;

% projection on YZ plane
cm = normalize((HrYZ*center));  % center
p1m = normalize(HrYZ*p1);       % P1
p2m = normalize(HrYZ*p2r);       % P2
pmm = normalize(HrYZ*pm);       % midpoint
% projection on XY plane
pmx = normalize(HrXY*pm);       % midpoint
cmx = normalize(HrXY*center);   % center

diffz = cm-pmm;
diffx = cmx-pmx;

if diffz<=0
    z = 1;
else
    z = -1;
end
if diffx(1)>=0
    x = 1;
else
    x = -1;
end

% distance between each point to midpoint in YZ plane
d1 = pdist([p1m(1:2).'; pmm(1:2).'], 'euclidean');
d2 = pdist([p2m(1:2).'; pmm(1:2).'], 'euclidean');
d = (d1+d2)/2;  % media of distances
% distance between midpoint to center in YZ plane
dz = pdist([cm(1:2).'; pmm(1:2).'], 'euclidean');
% distance between midpoint to center in XY plane
dx = pdist([cmx(1:2).'; pmx(1:2).'], 'euclidean');

% plot on 2D graph
figure(position2d)
hold on
plot(p1(1), p1(2),'or','MarkerSize',12, 'color', 'g');
plot(p2r(1), p2r(2),'or','MarkerSize',12, 'color', 'g');
plot(pm(1), pm(2),'or','MarkerSize',12, 'color', 'b');

disp('press key to continue'),pause % waiting key input
figure(position3d)
% plot on 3D graph
hold on
scatter3(x*dx, d, z*dz, 'g', 'filled')
scatter3(x*dx, -d, z*dz, 'g', 'filled')
scatter3(x*dx, 0, z*dz, 'b', 'filled')

P1 = [x*dx; d; z*dz; 1];
P2 = [x*dx; -d; z*dz; 1];
M = [x*dx; 0; z*dz; 1];
disp('press key to continue'),pause


end
