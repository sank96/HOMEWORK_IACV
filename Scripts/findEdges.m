function M = findCorners(imagesBW)
%FINDCORNERS Summary of this function goes here
%   Detailed explanation goes here

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

M = mapImage(GradNorm);
filterAdvanced(M, Gx, Gy);
return
end

