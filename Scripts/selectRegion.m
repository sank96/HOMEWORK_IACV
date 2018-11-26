function [imageRegion xR yR] = selectRegion(image, name)
%SELECTREGION Summary of this function goes here
%   Detailed explanation goes here
figure('Name', 'chose region');
imshow(image);

hRect = imrect();

% Rectangle position is given as [xmin, ymin, width, height]
pos_rect = hRect.getPosition();

% Round off so the coordinates can be used as indices
pos_rect = round(pos_rect);

% Select part of the image
imageRegion = image(pos_rect(2) + (0:pos_rect(4)), pos_rect(1) + (0:pos_rect(3)));
xR = [pos_rect(1) pos_rect(1)+pos_rect(3)    pos_rect(1)+pos_rect(3) pos_rect(1)];
yR = [pos_rect(2) pos_rect(2)                pos_rect(2)+pos_rect(4) pos_rect(2)+pos_rect(4)]; 


close 'chose region';

% name = strcat('region', number);

% figure('Name', name);
% imshow(imageRegion);

end

