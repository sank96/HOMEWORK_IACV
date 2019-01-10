function [C, profile, CX, CY] = findEllipses(image, name)
%FINDELLIPSES Usefull to find ellipses in an image
%   Given an image, it's selected a region in which are calculated every
%   possible ellipse. All of which are showed in a figure and user selects
%   the correct one. The function return:
%       C           : matrix of ellipse
%       PROFILE     : an image with the same size of the input image
%       CX & CY     : are the coordinates of points plotted that identify
%                     the ellipse

% select the region
[region, xW, yW] = selectRegion(image, name);
se1 = strel('disk', 4);       % to see the mask compute se1.Neighborhood
dil1 = imdilate(findEdges(region, 'canny'), se1);   % dilation
bw = dil1;

s = regionprops(bw,{...
    'Centroid',...
    'MajorAxisLength',...
    'MinorAxisLength',...
    'Orientation'});

figure('name', 'conic selection')
imshow(bw,'InitialMagnification','fit')

% array of 50 points from 0 to 2*pi
t = linspace(0,2*pi,50);
conics = [];    % array of conics
hold on
for k = 1:length(s)
    a = s(k).MajorAxisLength/2;
    b = s(k).MinorAxisLength/2;
    Xc = s(k).Centroid(1);
    Yc = s(k).Centroid(2);
    phi = deg2rad(-s(k).Orientation);
    x = Xc + a*cos(t)*cos(phi) - b*sin(t)*sin(phi);
    y = Yc + a*cos(t)*sin(phi) + b*sin(t)*cos(phi);
    if(a > 20 && b > 20)    % 20 - 20
        plot(x,y,'r','Linewidth',2)
        conics = [conics s(k)];
    end
end

% select the nearest point to the request conic
title('select the nearest point')
[xp, yp] = getpts;
plot(xp, yp, 'or', 'linewidth', 12)

% find the nearest conic
t = linspace(0,2*pi,50);
mindistances = zeros(length(conics),2);
for k = 1:length(conics)
    a = conics(k).MajorAxisLength/2;
    b = conics(k).MinorAxisLength/2;
    Xc = conics(k).Centroid(1);
    Yc = conics(k).Centroid(2);
    phi = deg2rad(-conics(k).Orientation);
    x = Xc + a*cos(t)*cos(phi) - b*sin(t)*sin(phi);
    y = Yc + a*cos(t)*sin(phi) + b*sin(t)*cos(phi);
    [distance, minX, minY] = nearest(xp, xp, x, y);
    mindistances(k,:) = [k distance];
end
mindistance = mindistances(find(mindistances(:,2)==min(mindistances(:,2))),:);

% selected conic
k = mindistance(1);
a = conics(k).MajorAxisLength/2;
b = conics(k).MinorAxisLength/2;
Xc = conics(k).Centroid(1);
Yc = conics(k).Centroid(2);
phi = deg2rad(-conics(k).Orientation);
x = Xc + a*cos(t)*cos(phi) - b*sin(t)*sin(phi);
y = Yc + a*cos(t)*sin(phi) + b*sin(t)*cos(phi);
plot(x,y,'b','Linewidth',2)

% plot the selected conic with different color
t1 = linspace(0,2*pi,6);
xT = Xc + a*cos(t1)*cos(phi) - b*sin(t1)*sin(phi);
yT = Yc + a*cos(t1)*sin(phi) + b*sin(t1)*cos(phi);
plot(xT, yT, 'or', 'linewidth', 12, 'color', 'g')


xW = xW(1);
yW = yW(1);
xT = (xT + xW).';
yT = (yT + yW).';

% points x and y of conic translated
CX = x+xW;
CY = y+yW;

% calculate C matrix from 5 points
A1=[xT.^2 xT.*yT yT.^2 xT yT ones(size(xT))];
N = null(A1);
cc = N(:, 1);
[a, b, c, d, e, f]=deal(cc(1),cc(2),cc(3),cc(4),cc(5),cc(6));
C=[a b/2 d/2; b/2 c e/2; d/2 e/2 f];
% C = C/C(3,3);
% return the profile of the conic on a binary image (0 background 1 conic)
profile = fromConicToProfile(image, C);
title('press return to close and continue')

disp('press key to continue'),pause
close('conic selection')

end
