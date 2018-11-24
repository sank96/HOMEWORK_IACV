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

% figure('Name', 'Image');
% imshow(images);

%% gray scale
imagesBW = rgb2gray(images);
% figure('Name', 'ImageBW');
% imshowpair(images, imagesBW, 'montage');

showTwoImages(images, imagesBW, 'ImageBW');

%% Grafico dell'immagine
% figure(3), bar3(imagesBW),title('kernel')

%% Find Edges
M = findEdges(imagesBW);
imageFiltered = filterThreshold(M, 0.2);
showTwoImages(M, imageFiltered, 'ImageBW_Filtered');
showTwoImages(M, imagesBW, 'ImageBW_M');
imagesEdge = edge(imagesBW, 'Sobel', []);
showTwoImages(imagesEdge, imagesBW, 'ImageBW_Edges');

%% Filter images
% imageFiltered = filterThreshold(images, 100);
