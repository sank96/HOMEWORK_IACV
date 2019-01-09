function [distance,pointX, pointY] = nearest(x,y,setX,setY)
%NEAREST Summary of this function goes here
%   Detailed explanation goes here
A = [x, y];
B = [setX.' setY.'];
distances = sqrt(sum(bsxfun(@minus, B, A).^2,2));
closest = B(find(distances==min(distances)),:);
distance = min(distances);
pointX = closest(1);
pointY = closest(2);
end

