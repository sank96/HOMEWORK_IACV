function M = findEdges(imagesBW, threshold)
%FINDCORNERS You can calculate the edges in your image setted the alghoritm
%to use
%   FINDCORNERS(imagesBW, threshold) return ad immagine in which edges are
%   highlighted. The implemented alghoritms are:
%   - 'binary' threshold
%   - 'hard' threshold  

%% Building filters as smoothing + differentiating filters

%disp('differentiating filters')
diffx=[1 -1];
diffy = diffx';

%smoothing filters Previtt
sx=ones(2,3);
sy=sx';

% build Previtt derivative filters
%disp('derivative filters Previtt')
dxP=conv2(sy , diffx);
dyP=conv2(sx , diffy);

%smoothing filters Sobel
sx=[1 2 1 ; 1 2 1];
sy=sx';

% Build Sobel derivative filters
%disp(' derivative filters Sobel')
dxS=conv2(sy,diffx);
dyS=conv2(sx,diffy);


%% Differentiating Filters

% compute gradient components (horizontal and vertical derivatives)
Gx=conv2(imagesBW , dxS , 'same');
Gy=conv2(imagesBW , dyS , 'same');

%figure(3),imshow(Gx, []),title('horizontal derivative')
%figure(4),imshow(Gy, []),title('vertical derivative')

% Gradient Norm
GradNorm=sqrt(Gx.^2 + Gy.^2);

BORDER = 3;
% remove boundaries as these are affected by zero padding
GradNorm(1 : BORDER, :) = 0; 
GradNorm(end - BORDER : end, :) = 0; 
GradNorm(:, 1 : BORDER) = 0; 
GradNorm(:, end - BORDER : end) = 0;

%% qui aggiungere altre possibili azioni in base alla threshold
if strcmp(threshold, 'hard')
    imageFiltered = filterHardThreshold(GradNorm);
elseif strcmp(threshold, 'binary')
    imageFiltered = filterBinaryThreshold(GradNorm);
elseif strcmp(threshold, 'canny')
    imageFiltered = edge(imagesBW, 'canny');
end

M = mapImage(imageFiltered);

end

