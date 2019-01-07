function [] = plotSymP(imagesBW, center, HrXY, HrYZ, position2d, position3d, vpY)
%PLOTSYMP Summary of this function goes here
%   Detailed explanation goes here
% get a pair of symmetric points and plot them 2d and 3d
[selection, xS, yS]  = selectRegion(imagesBW, 'symmetric area');
xS = xS(1);
yS = yS(1);

figure('Name', 'symmetric points');
imshow(findEdges(selection, 'binary'));
title('select a pair of symmetric points')
hold on;
[x, y]=getpts;
close('symmetric points')
p1 = [x(1)+xS; y(1)+yS; 1];
p2 = [x(2)+xS; y(2)+yS; 1];
pm = crossRatioMid(p1, p2, vpY);
cm = normalize((HrYZ*center));
p1m = normalize(HrYZ*p1);
p2m = normalize(HrYZ*p2);
pmm = normalize(HrYZ*pm);
pmx = normalize(HrXY*pm);
cmx = normalize(HrXY*center);

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

d1 = pdist([p1m(1:2).'; pmm(1:2).'], 'euclidean');
d2 = pdist([p2m(1:2).'; pmm(1:2).'], 'euclidean');
d = (d1+d2)/2;
dz = pdist([cm(1:2).'; pmm(1:2).'], 'euclidean');
dx = pdist([cmx(1:2).'; pmx(1:2).'], 'euclidean');

figure(position2d)
hold on
plot(p1(1), p1(2),'or','MarkerSize',12, 'color', 'g');
plot(p2(1), p2(2),'or','MarkerSize',12, 'color', 'g');
plot(pm(1), pm(2),'or','MarkerSize',12, 'color', 'b');

pause
figure(position3d)
hold on
scatter3(x*dx, -d, z*dz, 'g', 'filled')
scatter3(x*dx, d, z*dz, 'g', 'filled')
scatter3(x*dx, 0, z*dz, 'b', 'filled')


end

