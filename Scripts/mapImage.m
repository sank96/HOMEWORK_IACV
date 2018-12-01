function imageScaled = mapImage(imageOriginal)
%MAPIMAGE Summary of this function goes here
%   Detailed explanation goes here



% non serve xk basta caricarla come double!!!!
maxI = max(max(imageOriginal));
minI = min(min(imageOriginal));



imageScaled = imageOriginal*1 ./ maxI;

return  
end

