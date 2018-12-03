function [C, profile] = findWheel(image, name)
%FINDWHEEL Summary of this function goes here
%   Detailed explanation goes here
[wheelSelection, xW, yW]  = selectRegion(image, name);
% xW e yW contengono i vertici del rettangolo selezionato
xW = xW(1);
yW = yW(1);

wheelM = findEdges(wheelSelection, 'binary');

% display the input image in order to choose 5 points
figure('Name', 'find conic');
imshow(wheelM);
title('select 5 point')
hold on;
disp('click 5 points, then enter');
[x, y]=getpts;
scatter(x,y,100,'filled');

xT = x + xW;
yT = y + yW;

C = findConic(xT, yT);

[r, c] = size(image);
im = zeros(r, c);
for i=1:r
    for j=1:c
        im(i,j)=[j i 1]*C*[j i 1]';
    end
end

im = im < 0;
profile = findEdges(im, 'binary');

close 'find conic';
end

