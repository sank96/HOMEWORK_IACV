function imageProfiled = showProfileOnImage(image,imgP,x0,y0)
%SHOWPROFILE Allow to show a profile expressed by a binary image on another
%image
%   SHOWPROFILE(image, imgP, x0, y0) return an image that is a merge of ad
%   image (\image) and a boundary (\imgP). In order to mix images of
%   different size x0 and y0 indicate the origin (croner top left) of the 
%   image \imgP in the image \image


imageProfiled = image;
[R, C] = size(imgP);
for r = 1:R
    for c = 1:C
        if imgP(r,c) == 1
            imageProfiled(r+y0, c+x0) = 1;
        end
    end
end


end

