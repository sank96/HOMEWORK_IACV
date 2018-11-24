function imagesFilteres = filterAdvanced(image, Gx, Gy)
%FILTERADVANCED Summary of this function goes here
%   Detailed explanation goes here


% build the gradient image as a two plane image: the first plane contains horizontal derivatives, the second plane the % vertical ones
Grad=zeros(size(image,1),size(image,2),2);
Grad(:,:,1)=Gx;
Grad(:,:,2)=Gy;

% compute Gradient norm
Norm_Grad = sqrt(Grad(: , : , 1) .^ 2 + Grad(: , : , 2) .^ 2); BORDER = 3;

% remove boundaries as these are affected by zero padding
Norm_Grad(1 : BORDER, :) = 0; 
Norm_Grad(end - BORDER : end, :) = 0; 
Norm_Grad(:, 1 : BORDER) = 0; 
Norm_Grad(:, end - BORDER : end) = 0;

% we change the sign of the derivative because the y axis is "increasing downwards", since in Matlab it correspodns to the row index
Dir_Grad=atand(- sign(Grad(:,:,1)).*Grad(:,:,2) ./(abs(Grad(:,:,1))+eps));
figure(12),imshow(Norm_Grad,[]),title('norm'); 
figure(13),imshow(Dir_Grad,[]),title('directions');


% Binary Threshold - Play on the choice of the Threshold T
T = 5 * median(Norm_Grad(:));

% or choose the treshold using Otzu method (which is however meant for grayscale images, that are 
% assumed to contain two classes of pixels, thus that their intensity follow a bi-modal histogram) 
% [T , eff] = graythresh(Norm_Grad(:));
% arbitrary set the threshold
% T = quantile(Norm_Grad(:), 0.9); % 0.9 percentile
% compute the "gradient mask" by thresholding gradient magnitude

mask = Norm_Grad > T;
figure(14), imshow(mask), title('gradient mask')
% compute the histogram of gradient norm intensities
[hh , hh_bins] = hist(Norm_Grad(:) , 1000);
% show the histogram with the tresholded value
figure(7), bar(hh_bins , hh)
hold on
plot([T , T] , [min(hh) , max(hh)] , 'g' ,'LineWidth' , 3) 
hold off
% perform Hard Tresholding using a mask
Norm_Grad_Binary = double(Norm_Grad > T); Norm_Grad_Hard=Norm_Grad .* Norm_Grad_Binary;
% show Gradient Norm
figure(2),imshow(Norm_Grad,[]),title('Gradient norm');
% show the mask
figure(3),imshow(Norm_Grad_Binary,[]),title('Binary Thresholded gradient norm'); % show HT results
figure(4),imshow(Norm_Grad_Hard,[]),title(' Hard Thresholded gradient norm');
% show gradient directions in areas where gradient norm is large (i.e. over the gradient mask) figure(5),imshow(Norm_Grad_Binary.*Dir_Grad,[])
end

