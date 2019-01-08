function [imageRegion, xR, yR] = selectRegion(image, name)
%SELECTREGION Return a rectangular selection of an image and its Xmin and Ymin
%coordinates
%   SELECTREGION(image, name) show image in a new temporary figure. 
%   User is allowed to select a rectangular section of \image. 
%   The function return the section of the image \imageRegion and 
%   the coordinates of the origin of the region selected (its Xmin -> xR and Ymin -> yR)

% display a temporary figure in order to select the area
figure('Name', 'chose region');
imshow(image);
title(name);

hRect = imrect();

% Rectangle position is given as [xmin, ymin, width, height]
pos_rect = hRect.getPosition();

% Round off so the coordinates can be used as indices
pos_rect = round(pos_rect);

% Select part of the image
imageRegion = image(pos_rect(2) + (0:pos_rect(4)), pos_rect(1) + (0:pos_rect(3)));
xR = [pos_rect(1)   pos_rect(1)+pos_rect(3)    pos_rect(1)+pos_rect(3)      pos_rect(1)];
yR = [pos_rect(2)   pos_rect(2)                pos_rect(2)+pos_rect(4)      pos_rect(2)+pos_rect(4)]; 

% Close temporary figure precedently opened
close 'chose region';

end

