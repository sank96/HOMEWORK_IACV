function points = intersection(conic,lines)
%INTERSECTION Summary of this function goes here
%   Detailed explanation goes here

points = [];

for i = 1:length(lines)
    for j = 1:length(lines)
        l1 = lines(:,i);
        l2 = lines(:,j);
        if ~isequal(l1, l2)
            v = cross(l1, l2);
            v = v/v(3);
            if isOn(v, conic)    
%                 disp('trovato')
                points = [points v];
            end
        end
    end
end
points = unique(points.', 'row', 'stable').';
end

