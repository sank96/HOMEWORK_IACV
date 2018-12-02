%% reload
clc
clear all
close all

%% load image

image1 = im2double(imread('Image1.jpeg'));
image2 = im2double(imread('Image2.jpeg'));


imagesBW = rgb2gray(image1);

%% MAKE THE PICTURE SQUARED

% [ver_size hor_size] = size(imagesBW);
% del = (hor_size-ver_size)/2 ;
% imagesBW=imagesBW(:, del: (hor_size-del)-1);
% figure, imshow(imagesBW);


%% custom grayScale filter
% imagesBWFiltered = 0.299*images(:,:,1) + 0.587*images(:,:,2) + 0.114*images(:,:,3);
% showTwoImages(images, imagesBW, 'ImageBWFiltered');

%% comparison BW
% showTwoImages(imagesBW, imagesBWFiltered, 'comparisonBW');

%% Grafico dell'immagine
% imageScaled = imresize(imagesBW, 0.5);
% mesh(imageScaled);

%% seleziono la prima ruota
[wheel1, xW1, yW1]  = selectRegion(imagesBW, 'wheel1');
xW1 = xW1(1);
yW1 = yW1(1);

% con questi funziona
% wheel1 = imagesBW;
% xW1 = 0;
% yW1 = 0;

% EVIDENZIAMO LA PRIMA RUOTA

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
xW2 = xW2(1);
yW2 = yW2(1);

% con questi funziona
% wheel2 = imagesBW;
% xW2 = 0;
% yW2 = 0;

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
imageC = showProfile(imagesBW, wheel1Profile, xW1, yW1);
% showTwoImages(imagesBW, imageC, 'image wheels');
imageC = showProfile(imageC, wheel2Profile, xW2, yW2);
showTwoImages(imagesBW, imageC, 'image wheels');

%% find automatically ellipses

% [wheel2, xW2, yW2]  = selectRegion(imagesBW, 'wheel2');
% s = findEllipses(imagesBW);

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
x = double(sol.a).';
y = double(sol.b).';
onesV = ones(4, 1).';

lines = [x; y; onesV];

line1 = lines(:,1);
line2 = lines(:,2);
line3 = lines(:,3);
line4 = lines(:,4);

%% Tangenti CORRETTE
% se non si usano la funzione selectRegion funziona tutto!

linesP = findLines(imagesBW, lines);

linesP = showProfile(linesP, wheel1Profile, xW1, yW1);
linesP = showProfile(linesP, wheel2Profile, xW2, yW2);

figure, imshow(linesP);

%% Risolviamo con la funzione

R1 = [1 0 -xW1; 0 1 -yW1; 0 0 1];
R2 = [1 0 -xW2; 0 1 -yW2; 0 0 1];

C1prime = R1*wheel1C;
C2prime = R1*wheel2C;

%% troviamo le bitangenti
syms a b;

C1 = inv(C1prime);
C2 = inv(C2prime);
l = [a; b; 1];

% A = sym('A%d%d', [2 4])

% A1 = a^2*C1(1,1) + b^2*C1(2,2) + C1(3,3) + 2*a*b*C1(1,2) + 2*a*C1(1,3) + 2*b*C1(2,3);
% A2 = a^2*C2(1,1) + b^2*C2(2,2) + C2(3,3) + 2*a*b*C2(1,2) + 2*a*C2(1,3) + 2*b*C2(2,3);

A1 = l.' * C1 * l;
A2 = l.' * C2 * l;
sol = solve([A1 A2], [a b]);

% convert symbolic values into variables with double precision
% https://it.mathworks.com/help/symbolic/double.html
x = double(sol.a).';
y = double(sol.b).';
onesV = ones(4, 1).';

lines = [x; y; onesV];

line1 = lines(:,1);
line2 = lines(:,2);
line3 = lines(:,3);
line4 = lines(:,4);

linesP = findLines(imagesBW, lines);

linesP = showProfile(linesP, wheel1Profile, xW1, yW1);
linesP = showProfile(linesP, wheel2Profile, xW2, yW2);

figure(50), imshow(linesP);
%%
% close 5
figure(5), imshow(black);
points1 = [cross(line1, line2) cross(line1, line3) cross(line1, line4)];
points1 = points1./points1(3,:);

points2 = [cross(line2, line3) cross(line2, line4)];
points2 = points2./points2(3,:);

points3 = [cross(line3, line4)];
points3 = points3./points3(3,:);

hold on

% p3 = [points3(1) ; points3(2)];
% p3R = rotatePoint(p3, 90, black);

plot(points1(1,:), points1(2,:),'or','MarkerSize',12, 'color', 'g');
plot(points2(1,:), points2(2,:),'or','MarkerSize',12, 'color', 'r');
plot(points3(1,:), points3(2,:),'or','MarkerSize',12, 'color', 'y');


hold off

% [testC testP] = findConic(black);


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

a = [x(1); y(1); 1];
b = [x(2); y(2); 1];
c = [x(3); y(3); 1];
d = [x(4); y(4); 1];

% disegna linee fra punti!!!
ltest = [a.'; b.'; c.'; d.'; a.'];
line(ltest(:,1), ltest(:,2), 'LineWidth',2);

%% vertex
[R C] = size(imagesBW);
v1 = [1; 1; 1];
v2 = [C; 1; 1];
v3 = [C; R; 1];
v4 = [1; R; 1];

test1234 = rot90( imagesBW, 1);
figure(500);
imshow(test1234);

vertex = [v1 , v2, v3, v4];
close 100;
figure(100);
imshow(test1234);
hold on


% line([v1(1), v3(1)], [v1(2), v3(2)], 'LineWidth',2);
% line([v2(1), v4(1)], [v2(2), v4(2)], 'LineWidth',2);
plot(vertex(1,:), vertex(2,:),'or','MarkerSize',12);

% [x y]=getpts
% scatter(x,y,100,'filled');
% plot(vertex(:,1), vertex(:,2),'or','MarkerSize',12);
hold off

%% border
b1 = cross(v1, v2);
b1 = b1 / b1(3);

b2 = cross(v3, v2);
b2 = b2 / b2(3);

b3 = cross(v4, v3);
b3 = b3 / b3(3);

b4 = cross(v4, v1);
b4 = b4 / b4(3);


%% intersection

int11 = cross(b2, line1);
int11 = int11 / int11(3);
int21 = cross(b2, line2);
int21 = int21 / int21(3);
int31 = cross(b2, line3);
int31 = int31 / int31(3);
int41 = cross(b2, line4);
int41 = int41 / int41(3);

int12 = cross(b4, line1);
int12 = int12 / int12(3);
int22 = cross(b4, line2);
int22 = int22 / int22(3);
int32 = cross(b4, line3);
int32 = int32 / int32(3);
int42 = cross(b4, line4);
int42 = int42 / int42(3);

ints1 = [int11, int21, int31, int41];
ints2 = [int12, int22, int32, int42];
figure(100)
hold on
plot(ints1(1,:), ints1(2,:),'or','MarkerSize',12, 'Color', 'b');
plot(ints2(1,:), ints2(2,:),'or','MarkerSize',12, 'Color', 'g');

line([ints1(1,1), ints2(1,1)], [ints1(2,1), ints2(2,1)], 'LineWidth',2);
line([ints1(1,2), ints2(1,2)], [ints1(2,2), ints2(2,2)], 'LineWidth',2);
line([ints1(1,3), ints2(1,3)], [ints1(2,3), ints2(2,3)], 'LineWidth',2);
line([ints1(1,4), ints2(1,4)], [ints1(2,4), ints2(2,4)], 'LineWidth',2);
hold off


%% line

test123 = findLine(test, line4);
test123 = findLine(test123, line3);
figure(100);
imshow(test123);
