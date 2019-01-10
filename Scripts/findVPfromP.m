function [vp1, vp2, xT, yT] = findVPfromP(image, points)
%TEST Find vanishing points from car license plate
%   Detailed explanation goes here

% display the input image in order to choose the corner points
figure('name', 'select the corner of car license plate');
title('select the corner of car license plate');
imshow(image);
hold on;
plot(points(1,:), points(2,:), 'g+', 'LineWidth', 4, 'color', 'r')

% select the corner of the plate in this way
% 1   ------------  2
%    |            |
%    |            |
%    |            |
% 3   ------------  4
[x, y] = getpts;

% control there are selected 4 points
while length(x) ~= 4
    title('select the 4 corner')
    [x, y] = getpts;
end

ps = ones(3,4);
% find the nearest point from the choosen one
for i=1:length(x)
    [d, xp, yp] = nearest(x(i), y(i), points(1,:), points(2,:));
    ps(1:2, i) = [xp, yp];
end
% show the selected
plot(ps(1,:), ps(2,:), 'g+', 'LineWidth', 4, 'color', 'b')

% delay and then close the figure
pause
close('select the corner of car license plate')

xT = ps(1,:);
yT = ps(2,:);
points = ps;
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
