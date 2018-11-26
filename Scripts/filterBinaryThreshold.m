function Norm_Grad_Binary = filterBinaryThreshold(Norm_Grad)
%FILTERBINARYTHRESHOLD Summary of this function goes here
%   Detailed explanation goes here

% Binary Threshold - Play on the choice of the Threshold T
T = 5 * median(Norm_Grad(:));

% perform Hard Tresholding using a mask
Norm_Grad_Binary = double(Norm_Grad > T);
end

