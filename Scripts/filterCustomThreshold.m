function Norm_Grad_Hard = filterCustomThreshold(Norm_Grad, T)
%FILTERHARDTHRESHOLD Summary of this function goes here
%   Detailed explanation goes here

% perform Custom Tresholding using a mask
Norm_Grad_Binary = double(Norm_Grad > T); 
Norm_Grad_Hard=Norm_Grad .* Norm_Grad_Binary;
end

