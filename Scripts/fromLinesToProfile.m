function profile = fromLinesToProfile(image, lines)
%FINDLINES Return the profile of different lines
%   Detailed explanation goes here

L = size(lines);
L = L(2);
profile = zeros(size(image));
for l=1:L
    line = lines(:,l);
    
    profileL = fromLineToProfile(image, line);
    
    [R C] = size(image);
    
    for r=1:R
        for c=1:C
            if profileL(r,c) == 1
                profile(r,c)=1;
            end
        end
    end
end
end

