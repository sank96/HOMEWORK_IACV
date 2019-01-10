function im = fromConicToProfile(image,C)
%FROMCONICTOPROFILE Return the profile of the conic
%   Return an image of the same size of the input image, completely black 
%   except for the conic's points that are white

[r, c] = size(image);
im = zeros(r, c);
for i=1:r
    for j=1:c
        im(i,j)=[j i 1]*C*[j i 1]';
    end
end
% figure,imshow(im)
im = im < 0;
im = findEdges(im, 'binary');
end

