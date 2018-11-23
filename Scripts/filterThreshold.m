function imageFiltered = filterThreshold(image, threshold)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
heigth = length(image(:,1));
width  = length(image(1,:));

imageFiltered = zeros(heigth, width);

for r = 1:heigth
    for c = 1:width
        if image(r, c) > threshold
            imageFiltered(r, c) = 255;    
        else 
            imageFiltered(r, c) = 0;
        end
    end
end
         
end

