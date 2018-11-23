%% reload
clc
clear all
close all

%% load image

image1 = imread('Image1.jpeg');
image2 = imread('Image2.jpeg');

% modificare qui per decidere quali immagini visualizzare 
% se vuoi visualizzare più immagini mettile in riga
images = image1;

heigth = length(images(:,1,1));
width  = length(images(1,:,1));

% figure('Name', 'Image');
% imshow(images);

%% gray scale
imagesBW = rgb2gray(images);
% figure('Name', 'ImageBW');
% imshowpair(images, imagesBW, 'montage');

showTwoImages(images, imagesBW, 'ImageBW');




% %% Filtro B/W non funzionante 
% imagesR = images(:,:,1);
% imagesG = images(:,:,2);
% imagesB = images(:,:,3);
% 
% imshow([imagesR; imagesG; imagesB]);
% imagesBWFilter(heigth, width) = 0;
% 
% for R = 1:heigth
%     for C = 1:width
%         
%         imagesBWFilter(R, C) = (0.299*images(R,C,1) + 0.587*images(R,C,2) + 0.114*images(R,C,3))/255;
%         
%     end
% end
% 
% figure('Name', 'ImageBWFilter', );
% imshow(imagesBWFilter);

%% Grafico dell'immagine
% figure(3), bar3(imagesBW),title('kernel')

%% Find Corners
findCorners(imagesBW);

%% Filter images
% imageFiltered = filterThreshold(images, 100);
