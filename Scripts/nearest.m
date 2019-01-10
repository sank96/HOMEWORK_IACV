function [distance,pointX, pointY] = nearest(x,y,setX,setY)
%NEAREST find the nearest point to a set of point
%   The nereast point in the sense of minimum distance

A = [x, y];
B = [setX.' setY.'];
distances = sqrt(sum(bsxfun(@minus, B, A).^2,2));   % calculate the distance
closest = B(find(distances==min(distances)),:);     % find the point with minimum distance
distance = min(distances);      % inimum distance
pointX = closest(1);    
pointY = closest(2);
end

