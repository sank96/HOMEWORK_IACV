function showTwoImages(fig1,fig2, name)
%SHOWTWOIMAGES Summary of this function goes here
%   Detailed explanation goes here
figure('Name', name);
imshowpair(fig1, fig2, 'montage');
end

