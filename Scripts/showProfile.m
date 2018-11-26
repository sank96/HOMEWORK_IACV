function imageProfiled = showProfile(image,imgP,x0,y0)
%SHOWPROFILE Summary of this function goes here
%   Detailed explanation goes here
imageProfiled = image;
[R C] = size(imgP);
for r = 1:R
    for c = 1:C
        if imgP(r,c) == 1
            imageProfiled(r+y0, c+x0) = 255;
        
    end
end


end

