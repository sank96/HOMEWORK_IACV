function imageProfile = showProfileOpt(image,profile)
%SHOWPROFILEOPT Summary of this function goes here
%   Detailed explanation goes here

profileMask = profile==1;
[r,c] = find(profileMask);
imageProfile = image;
for i=1:length(r)
    imageProfile(r(i),c(i))=1;
end
end

