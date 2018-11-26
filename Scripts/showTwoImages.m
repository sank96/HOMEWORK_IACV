function showTwoImages(fig1,fig2, name)
%SHOWTWOIMAGES Display two images in a single figure named with \name
%   SHOWTWOIMAGES(fig1, fig2, name) The two images in input \fig1 and \fig2 are displayed with a
%   concatenation horizontaly in the same figure
figure('Name', name);
imshowpair(fig1, fig2, 'montage');
end

