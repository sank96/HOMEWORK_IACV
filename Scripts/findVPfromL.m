function [vp1, vp2, xT, yT] = findVPfromL(image)
%TEST Find vanishing points from car license plate
%   Detailed explanation goes here

% select the region that contains the car license plate
[clpSelection, xW, yW]  = selectRegion(image, 'car license');
xW = xW(1);
yW = yW(1);
clpM = findEdges(clpSelection, 'canny');

% display the input image in order to choose the corner points
figure('Name', 'find car license plate');
imshow(clpM);
title('select 4 corner points')
hold on;
[x, y]=getpts;
scatter(x,y,100,'filled');
xT = x + xW;
yT = y + yW;
close 'find car license plate';

points = [xT.' ; yT.' ; ones(1,length(xT))];

% find the vanishing point
vp1 = cross(...
    cross(points(:,1), points(:,2)),...
    cross(points(:,3), points(:,4)));
vp1 = vp1/vp1(3);
vp2 = cross(...
    cross(points(:,1), points(:,4)),...
    cross(points(:,2), points(:,3)));
vp2 = vp2/vp2(3);

end
% 
% % tangenti
% v1 = intersection(C1, lines);
% v1 = [v1(:,2) v1(:,1)]; % la prima colonna di entrambi i vettori di punti sono i punti in alto
% v2 = intersection(C2, lines);
% 
% imageProfile = fromLinesToProfile(image, [tan2 tan3]);
% imageProfile = showProfileOnImage(imageProfile, profile1, 0, 0);
% imageProfile = showProfileOnImage(imageProfile, profile2, 0, 0);
% img = showProfileOnImage(image, imageProfile, 0,0);
% 
% line1 = cross(v1(:,1), v2(:,1));
% line1 = line1/line1(3);
% line2 = cross(v1(:,2), v2(:,2));
% line2 = line2/line2(3);
% vpoint1 = cross(line1, line2);
% vpoint1 = vpoint1/vpoint1(3);
% 
% line3 = cross(v1(:,1), v1(:,2));
% line3 = line3/line3(3);
% line4 = cross(v2(:,1), v2(:,2));
% line4 = line4/line4(3);
% vpoint2 = cross(line3, line4);
% vpoint2 = vpoint2/vpoint2(3);
% 
% 
% 
