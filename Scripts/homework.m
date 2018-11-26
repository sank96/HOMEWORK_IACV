%% reload
clc
clear all
close all

%% load image

image1 = imread('Image1.jpeg');
image2 = imread('Image2.jpeg');

% modificare qui per decidere quali immagini visualizzare
% se vuoi visualizzare piu' immagini mettile in riga
images = image1;

% figure('Name', 'Image');
% imshow(images);

%% gray scale
imagesBW = rgb2gray(images);
showTwoImages(images, imagesBW, 'ImageBW');

%% custom grayScale filter
% imagesMapped = mapImage(images);
% imagesBWFiltered = 0.299*images(:,:,1) + 0.587*images(:,:,2) + 0.114*images(:,:,3);
%
% showTwoImages(images, imagesBW, 'ImageBWFiltered');

%% comparison BW
% showTwoImages(imagesBW, imagesBWFiltered, 'comparisonBW');

%% Grafico dell'immagine
% imageScaled = imresize(imagesBW, 0.5);
% mesh(imageScaled);

%% seleziono la prima ruota

[wheel1, xW1, yW1]  = selectRegion(imagesBW, 'wheel1');


%% evidenziamo la prima ruota

% i valori di threshold fin ora implementati sono
% 'hard' 'binary'
wheel1M = findEdges(wheel1, 'binary');
% wheel1MS = edge(wheel1, 'Sobel', []);
% wheel1MC = edge(wheel1, 'canny');

%% ricaviamo l'ellisse
[wheel1C, wheel1Profile] = findConic(wheel1M);
% showTwoImages(wheel1, wheel1Profile, 'profile wheel 1');

%% mostriamo profilo
wheel1P = showProfile(wheel1, wheel1Profile, 0, 0);
showTwoImages(wheel1, wheel1P, 'wheel 1 profile');




%% seleziono la seconda ruota

[wheel2, xW2, yW2]  = selectRegion(imagesBW, 'wheel2');

%% evidenziamo la seconda ruota

% i valori di threshold fin ora implementati sono
% 'hard' 'binary'
wheel2M = findEdges(wheel2, 'binary');

%% ricaviamo l'ellisse
[wheel2C, wheel2Profile] = findConic(wheel2M);
% showTwoImages(wheel1, wheel1Profile, 'profile wheel 1');

%% mostriamo profilo
wheel2P = showProfile(wheel2, wheel2Profile, 0, 0);
showTwoImages(wheel2, wheel2P, 'wheel 2 profile');

%% mostriamo ruote
imageC = showProfile(imagesBW, wheel1Profile, xW1(1), yW1(1));
showTwoImages(imagesBW, imageC, 'image wheels');
imageC = showProfile(imageC, wheel2Profile, xW2(1), yW2(1));
showTwoImages(imagesBW, imageC, 'image wheels');

