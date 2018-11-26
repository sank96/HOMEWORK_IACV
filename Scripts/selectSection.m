function [x, y] = selectSection(image,optionalStamp)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

figure('Name', 'chose section');
imshow(image);
hold on;
[x, y]=getpts;

%% nargin return the number of input parameters 
if nargin > 1
    if optionalStamp == true
        plot(x,y,'.w','MarkerSize',12, 'LineWidth', 3); % plots points clicked by user with red circles
    end
end
end
