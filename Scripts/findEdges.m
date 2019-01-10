function M = findEdges(imagesBW, threshold, value)
%FINDCORNERS You can calculate the edges in your image setted the alghoritm
%to use
%   FINDCORNERS(imagesBW, threshold) return ad immagine in which edges are
%   highlighted. The implemented alghoritms are:
%   - 'binary' threshold
%   - 'hard' threshold 
%   - 'custom'
%   - 'canny' 

diffx=[1 -1];
diffy = diffx';

%smoothing filters Sobel
sx=[1 2 1 ; 1 2 1];
sy=sx';

% Build Sobel derivative filters
%disp(' derivative filters Sobel')
dxS=conv2(sy,diffx);
dyS=conv2(sx,diffy);

% Differentiating Filters

% compute gradient components (horizontal and vertical derivatives)
Gx=conv2(imagesBW , dxS , 'same');
Gy=conv2(imagesBW , dyS , 'same');

% Gradient Norm
GradNorm=sqrt(Gx.^2 + Gy.^2);

BORDER = 3;
% remove boundaries as these are affected by zero padding
GradNorm(1 : BORDER, :) = 0; 
GradNorm(end - BORDER : end, :) = 0; 
GradNorm(:, 1 : BORDER) = 0; 
GradNorm(:, end - BORDER : end) = 0;

% added more thresold filters
if strcmp(threshold, 'hard')
    imageFiltered = filterHardThreshold(GradNorm);
elseif strcmp(threshold, 'binary')
    imageFiltered = filterBinaryThreshold(GradNorm);
elseif strcmp(threshold, 'canny')
    imageFiltered = edge(imagesBW, 'canny');
elseif strcmp(threshold, 'custom')
    imageFiltered = filterCustomThreshold(GradNorm, value);
end

M = imageFiltered;

end

