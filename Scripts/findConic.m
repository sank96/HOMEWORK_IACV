function [C, profile] = findConic(image, name, edges)
%FINDCONIC Summary of this function goes here
%   Detailed explanation goes here

% select the region that contains the wheel 
[wheelSelection, xW, yW]  = selectRegion(image, name);

% xW and yW are the factors of the tranlsation of points in the region
xW = xW(1);
yW = yW(1);

% compute edge detection if request
if nargin == 2 
    wheelM = findEdges(wheelSelection, 'binary');
else 
    wheelM = wheelSelection;
end

% display the input image in order to choose 5 points
figure('Name', 'find conic');
imshow(wheelM);
title('select 5 point')
hold on;
% disp('click 5 points, then enter');
[x, y]=getpts;
scatter(x,y,100,'filled');

xT = x + xW;
yT = y + yW;

% calculate C matrix from 5 points
A=[xT.^2 xT.*yT yT.^2 xT yT ones(size(xT))];
N = null(A);
cc = N(:, 1);
[a, b, c, d, e, f]=deal(cc(1),cc(2),cc(3),cc(4),cc(5),cc(6));
C=[a b/2 d/2; b/2 c e/2; d/2 e/2 f];

% close the temporary figure
close 'find conic';

% return the profile of the conic on a binary image (0 background 1 conic)
profile = fromConicToProfile(image, C);
% profile = zeros(size(image));
% [r, c]=size(sectionProfile);
% profile(yW:yW+r-1,xW:xW+c-1)=sectionProfile;


end

