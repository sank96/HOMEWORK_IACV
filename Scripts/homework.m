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
% showTwoImages(images, imagesBW, 'ImageBW');

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


% evidenziamo la prima ruota

% i valori di threshold fin ora implementati sono
% 'hard' 'binary'
wheel1M = findEdges(wheel1, 'binary');
% wheel1MS = edge(wheel1, 'Sobel', []);
% wheel1MC = edge(wheel1, 'canny');

% ricaviamo l'ellisse
[wheel1C, wheel1Profile] = findConic(wheel1M);
% showTwoImages(wheel1, wheel1Profile, 'profile wheel 1');

% mostriamo profilo
wheel1P = showProfile(wheel1, wheel1Profile, 0, 0);
% showTwoImages(wheel1, wheel1P, 'wheel 1 profile');




%% seleziono la seconda ruota

[wheel2, xW2, yW2]  = selectRegion(imagesBW, 'wheel2');

% evidenziamo la seconda ruota

% i valori di threshold fin ora implementati sono
% 'hard' 'binary'
wheel2M = findEdges(wheel2, 'binary');

% ricaviamo l'ellisse
[wheel2C, wheel2Profile] = findConic(wheel2M);
% showTwoImages(wheel1, wheel1Profile, 'profile wheel 1');

% mostriamo profilo
wheel2P = showProfile(wheel2, wheel2Profile, 0, 0);
% showTwoImages(wheel2, wheel2P, 'wheel 2 profile');

%% mostriamo ruote
imageC = showProfile(imagesBW, wheel1Profile, xW1(1), yW1(1));
% showTwoImages(imagesBW, imageC, 'image wheels');
imageC = showProfile(imageC, wheel2Profile, xW2(1), yW2(1));
showTwoImages(imagesBW, imageC, 'image wheels');

%% troviamo le bitangenti
syms a b;

C1 = inv(wheel1C);
C2 = inv(wheel2C);
l = [a; b; 1];

% A = sym('A%d%d', [2 4])

% A1 = a^2*C1(1,1) + b^2*C1(2,2) + C1(3,3) + 2*a*b*C1(1,2) + 2*a*C1(1,3) + 2*b*C1(2,3);
% A2 = a^2*C2(1,1) + b^2*C2(2,2) + C2(3,3) + 2*a*b*C2(1,2) + 2*a*C2(1,3) + 2*b*C2(2,3);

A1 = l.' * C1 * l;
A2 = l.' * C2 * l;
sol = solve([A1 A2], [a b]);

% convert symbolic values into variables with double precision
% https://it.mathworks.com/help/symbolic/double.html
x = double(sol.a);
y = double(sol.b);
onesV = ones(4, 1);

lines = [x y onesV];

line1 = lines(1, :).';
line2 = lines(2, :).';
line3 = lines(3, :).';
line4 = lines(4, :).';
% figure
% refline(line1(1), line1(2));
% refline(lines(:,1), lines(:,2));
%
% im_four=zeros(500,500);
% for i=1:500
%     for j=1:500
%         im_four(i,j)=[j i 1]*wheel1C*[j i 1]';
%     end
% end
% imshow(im_four > 0);
%
% hold off

%% test
test = zeros(500);
figure(50), imshow(test);
hold on
% keeps on drawing multiple elements on the same figure hold on;
% select six visible verteces of the cube
[x y]=getpts;
% scatter(x,y,100,'filled');
plot(x,y,'or','MarkerSize',12);
hold off

a = [x(1); y(1); 1]
b = [x(2); y(2); 1]
c = [x(3); y(3); 1]
d = [x(4); y(4); 1]

% disegna linee fra punti!!!
ltest = [a.'; b.'; c.'; d.'; a.'];
line(ltest(:,1), ltest(:,2), 'LineWidth',2);


%% line

test123 = findLine(test, line4);
test123 = findLine(test123, line3);
figure(100);
imshow(test123);
