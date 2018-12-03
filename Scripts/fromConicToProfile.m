function im = fromConicToProfile(image,C)
%FROMCONICTOPROFILE Summary of this function goes here
%   Detailed explanation goes here
[r, c] = size(image);
im = zeros(r, c);
for i=1:r
    for j=1:c
        im(i,j)=[j i 1]*C*[j i 1]';
    end
end
im = im < 0;
im = findEdges(im, 'binary');
end

