%% reload
clc
clear all
close all

%% load image

image1 = im2double(imread('Image1.jpeg'));
image2 = im2double(imread('Image2.jpeg'));

imagesBW = rgb2gray(image1);
% figure, imshow(imagesBW)

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

%% point 1
% TROVIAMO LE RUOTE MANUALMENTE

[C1, profile1] = findWheel(imagesBW, 'wheel1');

% mostriamo profilo
% imageWithWheels = showProfileOnImage(imagesBW, profile1, 0, 0);
imageWithWheels = showProfileOpt(imagesBW, profile1);

[C2, profile2] = findWheel(imagesBW, 'wheel2');
% mostriamo profilo
imageWithWheels = showProfileOnImage(imageWithWheels, profile2, 0, 0);

% showTwoImages(imagesBW, imageWithWheels, 'conic as wheels')

%% find automatically ellipses

% s = findEllipses(imagesBW);

%% point 2.1
% bitangenti
syms a b;

C1star = inv(C1);
C2star = inv(C2);


l = [a; b; 1];

% A = sym('A%d%d', [2 4])

% A1 = a^2*C1(1,1) + b^2*C1(2,2) + C1(3,3) + 2*a*b*C1(1,2) + 2*a*C1(1,3) + 2*b*C1(2,3);
% A2 = a^2*C2(1,1) + b^2*C2(2,2) + C2(3,3) + 2*a*b*C2(1,2) + 2*a*C2(1,3) + 2*b*C2(2,3);

A1 = l.' * C1star * l;
A2 = l.' * C2star * l;

sol = solve([A1 A2], [a b]);

% convert symbolic values into variables with double precision
% https://it.mathworks.com/help/symbolic/double.html
x = double(sol.a).';
y = double(sol.b).';
onesV = ones(4, 1).';

lines = [x; y; onesV];

tan1 = lines(:,1);
tan2 = lines(:,2);
tan3 = lines(:,3);
tan4 = lines(:,4);

% tangenti
v1 = intersection(C1, lines);
v1 = [v1(:,2) v1(:,1)]; % la prima colonna di entrambi i vettori di punti sono i punti in alto
v2 = intersection(C2, lines);

tic
linesP = fromLinesToProfile(imagesBW, [tan2 tan3]);
toc

linesP = showProfileOnImage(linesP, profile1, 0, 0);
linesP = showProfileOnImage(linesP, profile2, 0, 0);

img = showProfileOnImage(imagesBW, linesP, 0,0);
linesFigure = figure();
imshow(img);


figure(linesFigure)
hold on
plot(v1(1,:), v1(2,:), 'or','MarkerSize',12, 'color', 'g');
plot(v2(1,:), v2(2,:), 'or','MarkerSize',12, 'color', 'r');
hold off

% BACK TRANSFORMATION
figure(linesFigure)
hold on
line1 = cross(v1(:,1), v2(:,1));
line1 = line1/line1(3);
line2 = cross(v1(:,2), v2(:,2));
line2 = line2/line2(3);
vpoint1 = cross(line1, line2);
vpoint1 = vpoint1/vpoint1(3);

line3 = cross(v1(:,1), v1(:,2));
line3 = line3/line3(3);
line4 = cross(v2(:,1), v2(:,2));
line4 = line4/line4(3);
vpoint2 = cross(line3, line4);
vpoint2 = vpoint2/vpoint2(3);


plot(vpoint1(1), vpoint1(2), 'or','MarkerSize',12, 'color', 'c');
plot(vpoint2(1), vpoint2(2), 'or','MarkerSize',12, 'color', 'c');

lineInf = cross(...                 % line at infinity
    cross(line1, line2), ...        % vanishing point
    cross(line3, line4));          % vanishing point
lineInf = lineInf/lineInf(3);

syms x y
xVector = [x; y; 1];

A1 = lineInf.' * xVector;
A2 = xVector.' * C1 * xVector;

sol = solve([A1 A2], [x y]);

% convert symbolic values into variables with double precision
% https://it.mathworks.com/help/symbolic/double.html
circularPoint =[double(sol.x).'; double(sol.y).'; ones(1,length(double(sol.x)))];
I = circularPoint(:,1);
J = circularPoint(:,2);

plot(I(1), I(2), 'or','MarkerSize',12, 'color', 'b');
plot(J(1), J(2), 'or','MarkerSize',12, 'color', 'b');

Cinf = I*J' + I'*J;
[U,S,V] = svd(Cinf);            % A = U*S*V'

s11 = S(1,1);
s22 = S(2,2);
T = [ sqrt(s11) 0 0 ; 0 sqrt(s22) 0 ; 0 0 0];

Hr = T * U;

hold off

% RECTIFICATION 

v1backTra = inv(Hr) * v1;
v1backTra = [v1backTra(:,1)/v1backTra(3,1) v1backTra(:,2)/v1backTra(3,2)];
v2backTra = inv(Hr) * v2;
v2backTra = [v2backTra(:,1)/v2backTra(3,1) v2backTra(:,2)/v2backTra(3,2)];

diameter =  pdist([v1(1,1) v1(1,2);v1(1,2) v1(2,2)],'euclidean')
distancewtow = pdist([v1(1,1) v2(1,1);v1(1,2) v2(1,2)],'euclidean')
diameter1 = pdist([v2(1,1) v2(1,2);v2(1,2) v2(2,2)],'euclidean')

ratio = diameter/distancewtow
ratio1 = diameter1/distancewtow

%% point 2.2 


