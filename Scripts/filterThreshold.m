function imageFiltered = filterThreshold(image, threshold)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
[R C] = size(image);

imageFiltered = zeros(R, C);

for r = 1:R
    for c = 1:C
        if image(r, c) > threshold
            imageFiltered(r, c) = 1;    
        else 
            imageFiltered(r, c) = 0;
        end
    end
end
return        
end

