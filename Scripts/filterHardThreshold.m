function Norm_Grad_Hard = filterHardThreshold(Norm_Grad)
%FILTERHARDTHRESHOLD Summary of this function goes here
%   Detailed explanation goes here

% Binary Threshold - Play on the choice of the Threshold T
T = 5 * median(Norm_Grad(:));

% or choose the treshold using Otzu method (which is however meant for grayscale images, that are 
% assumed to contain two classes of pixels, thus that their intensity follow a bi-modal histogram) 
% [T , eff] = graythresh(Norm_Grad(:));

% arbitrary set the threshold
% T = quantile(Norm_Grad(:), 0.9); % 0.9 percentile

% compute the "gradient mask" by thresholding gradient magnitude
mask = Norm_Grad > T;

% compute the histogram of gradient norm intensities
[hh , hh_bins] = hist(Norm_Grad(:) , 1000);

% show the histogram with the tresholded value
figure(7), bar(hh_bins , hh)
hold on
plot([T , T] , [min(hh) , max(hh)] , 'g' ,'LineWidth' , 3) 
hold off

% perform Hard Tresholding using a mask
Norm_Grad_Binary = double(Norm_Grad > T); 
Norm_Grad_Hard=Norm_Grad .* Norm_Grad_Binary;
end

