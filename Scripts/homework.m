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

[C1, profile1] = findConic(imagesBW, 'wheel1');

% mostriamo profilo
% imageWithWheels = showProfileOnImage(imagesBW, profile1, 0, 0);
imageWithWheels = showProfileOpt(imagesBW, profile1);

[C2, profile2] = findConic(imagesBW, 'wheel2');
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
linesFigure = figure('name', 'visualization');
imshow(img);


figure(linesFigure)
hold on
plot(v1(1,:), v1(2,:), 'or','MarkerSize',12, 'color', 'g');
plot(v2(1,:), v2(2,:), 'or','MarkerSize',12, 'color', 'g');
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


plot(vpoint1(1), vpoint1(2), 'or','MarkerSize',12, 'color', 'r');
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

plot(I(1), I(2), 'or','MarkerSize',12, 'color', 'y');
plot(J(1), J(2), 'or','MarkerSize',12, 'color', 'y');

Cinf = I*J' + J*I';
[U,S,V] = svd(Cinf);            % A = U*S*V'

s11 = S(1,1);
s22 = S(2,2);
T = [ sqrt(s11)     0       0 ;...
        0       sqrt(s22)   0 ;...
        0           0       1];

bm = U*T;
Hr=inv(bm);
Hr=Hr/Hr(3,3);

hold off

% RECTIFICATION
bHr = Hr;

v1backTra = bHr*v1;
v1bt = [v1backTra(1:2,1)/v1backTra(3,1) v1backTra(1:2,2)/v1backTra(3,2)];
v2backTra = bHr*v2;
v2bt = [v2backTra(1:2,1)/v2backTra(3,1) v2backTra(1:2,2)/v2backTra(3,2)];

% diameter = sqrt(...
%     (v1bt(1,1)-v1bt(1,2))*(v1bt(1,1)-v1bt(1,2)) +...
%     (v1bt(2,1)-v1bt(2,2))*(v1bt(2,1)-v1bt(2,2)))
% diameter1 = sqrt(...
%     (v2bt(1,1)-v2bt(1,2))*(v2bt(1,1)-v2bt(1,2)) +...
%     (v2bt(2,1)-v2bt(2,2))*(v2bt(2,1)-v2bt(2,2)))

diameter =  pdist([v1bt(1,1) v1bt(2,1);v1bt(1,2) v1bt(2,2)],'euclidean');
distancewtow = pdist([v1bt(1,1) v1bt(1,2);v2bt(1,1) v2bt(1,2)],'euclidean');
diameter1 = pdist([v2bt(1,1) v2bt(2,1);v2bt(1,2) v2bt(2,2)],'euclidean');

ratio = diameter/distancewtow
ratio1 = diameter1/distancewtow

%% point 2.2
% find the image of absolute conic

% %find other 2 vanishing points from sensors
% [vps1, vps2, profile, ps1, ps2] = findVPfromC(img);
% 
% figure(linesFigure)
% % imshow(profile)
% hold on
% plot(ps1(1,:), ps1(2,:), 'or','MarkerSize',12, 'color', 'g');
% plot(ps2(1,:), ps2(2,:), 'or','MarkerSize',12, 'color', 'r');
% plot(vps1(1), vps1(2), 'or','MarkerSize',12, 'color', 'c');
% plot(vps2(1), vps2(2), 'or','MarkerSize',12, 'color', 'c');
% hold off

% find vanishing point from the car license plate
[vps1, vps2, xT, yT] = findVPfromL(imagesBW);
figure(linesFigure)
% imshow(imagesBW);
hold on 
plot(xT, yT, 'or','MarkerSize',12, 'color', 'g');
plot(vps1(1), vps1(2), 'or','MarkerSize',12, 'color', 'b');
plot(vps2(1), vps2(2), 'or','MarkerSize',12, 'color', 'y');
hold off

% camera matrix
syms fxI fyI u0I v0I;

KI = [fxI 0 u0I;...
    0 fyI v0I;...
    0 0 1];
wstar = KI*KI.';
w = inv(wstar);

eq1 = (vpoint1.')*w*vpoint2;
eq2 = (vpoint1.')*w*vps1;
eq3 = (vpoint2.')*w*vps1;
eq4 = (I.')*w*I;

sol = solve([eq1 eq2 eq3 eq4], [fxI fyI u0I v0I]);

fx = double(sol.fxI);
fy = double(sol.fyI);
u0 = double(sol.u0I);
v0 = double(sol.v0I);

fx = abs(fx(1));
fy = abs(fy(1));
u0 = abs(u0(1));
v0 = abs(v0(1));

K = [   fx  0   u0 ;...
        0   fy  v0 ;...
        0   0   1]
iac = inv(K*K.');       % w 
iac = iac/iac(3,3);


%% point 2.3 
close all
vpointX = vpoint1;
vpointY = vps1;
vpointZ = vpoint2;
w = iac;

position2d = figure('name', '2d position');
imshow(imagesBW);
hold on
% plot(xT(3:4), yT(3:4),'or','MarkerSize',12, 'color', 'g');

HrYZ = backTransformation(vpointY, vpointZ, w);
HrXY = backTransformation(vpointX, vpointY, w);
HrXZ = backTransformation(vpointX, vpointZ, w);

p = [ HrYZ * [xT(3) ; yT(3) ; 1] ...
    HrYZ * [xT(4) ; yT(4) ; 1]];
p = [ p(:,1)/p(3,1) p(:,2)/p(3,2)];
cp = (p(:,1)+p(:,2))/2;
center = inv(HrYZ) * cp;
% punto medio della targa in basso

plot(center(1), center(2),'or','MarkerSize',12, 'color', 'r');

position3d = figure('name', '3d position');
scatter3(0, 0, 0)
%% 
% close all
% T = maketform('projective',[v1(1:2,2)';v2(1:2,2)';v2(1:2,1)';v1(1:2,1)'],[ 100,100 ;0,100; 0 0; 100, 0]); 
% figure, imshow(flipdim(imtransform(imagesBW,T),2));
%% 
get3dPosition([xT(1);yT(2);1], center, HrXY, HrYZ, HrXZ)