function [P1, P2, p1, p2, M, m] = plotSymP(imagesBW, center, HrXY, HrYZ, position2d, position3d, vpY)
%PLOTSYMP Summary of this function goes here
%   Detailed explanation goes here
% get a pair of symmetric points and plot them 2d and 3d

% select area conteining symmetric points
[selection, xS, yS]  = selectRegion(imagesBW, 'symmetric area');
xS = xS(1);
yS = yS(1);

pause(0.3)
figure('Name', 'symmetric points');
imshow(findEdges(selection, 'canny'));
title('select a pair of symmetric points')
hold on;
[x, y]=getpts;
close('symmetric points')

% symmetric points
p1 = [x(1)+xS; y(1)+yS; 1];
p2 = [x(2)+xS; y(2)+yS; 1];
% midpoint
pm = crossRatioMid(p1, p2, vpY);
m = pm;

% projection on YZ plane
cm = normalize((HrYZ*center));  % center
p1m = normalize(HrYZ*p1);       % P1
p2m = normalize(HrYZ*p2);       % P2
pmm = normalize(HrYZ*pm);       % midpoint
% projection on XY plane
pmx = normalize(HrXY*pm);       % midpoint
cmx = normalize(HrXY*center);   % center

if cm-pmm<=0
    z = 1;
else 
    z = -1;
end
if cmx-pmx<=0
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
plot(p2(1), p2(2),'or','MarkerSize',12, 'color', 'g');
plot(pm(1), pm(2),'or','MarkerSize',12, 'color', 'b');

pause % waiting key input
figure(position3d)
% plot on 3D graph
hold on
scatter3(x*dx, -d, z*dz, 'g', 'filled')
scatter3(x*dx, d, z*dz, 'g', 'filled')
scatter3(x*dx, 0, z*dz, 'b', 'filled')

P1 = [x*dx; -d; z*dz; 1];
P2 = [x*dx; d; z*dz; 1];
M = [x*dx; 0; z*dz; 1];

end

