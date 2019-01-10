function imageProfile = showProfileOpt(image,profile)
%SHOWPROFILEOPT show the input profile on the input image
%   With optymized function the profile is put over the original matrix as
%   white pixel. Is not necessary to inspect all the matrix to change its
%   pixels but it is done a targeted research fo mixels that must be
%   changed

profileMask = profile==1;
[r,c] = find(profileMask);  % find pixel to modify
imageProfile = image;
for i=1:length(r)
    imageProfile(r(i),c(i))=1;
end
end

