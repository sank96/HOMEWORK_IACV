function profile = findLine(image,lines)
%FINDLINE Lines is a matrix in which each line corresponds to a row of the matrix
%   Detailed explanation goes here
linesInv = lines.';
[R C] = size(image);
profile = zeros(R, C);
L = length(linesInv);
for r=1:R
    for c=1:C
        for i=1:L
            line = linesInv(i,:);
            profile(r,c)=[c r 1]*line;
        end
    end
end
profile=double(profile<0.002 & profile>-0.002);
end
