function profile = fromLineToProfile(image,line)
%FINDLINE Return the profile of line on the image
%   Detailed explanation goes here

[R C] = size(image);
profile = zeros(R, C);
for r=1:R
    for c=1:C
        profile(r,c)=[c r 1]*line;
    end
end
profile = double(profile>0);
profile = findEdges(profile, 'binary');
% profile=double(profile<0.002 & profile>-0.002);
end
