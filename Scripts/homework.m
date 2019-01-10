%% reload
clc
clear all
close all

%% load image

image1 = im2double(imread('Image1.jpeg'));
image2 = im2double(imread('Image2.jpeg'));

imagesBW = rgb2gray(image1);
% figure, imshow(imagesBW)

%% custom grayScale filter
% imagesBWFiltered = 0.299*images(:,:,1) + 0.587*images(:,:,2) + 0.114*images(:,:,3);
% showTwoImages(images, imagesBW, 'ImageBWFiltered');
% comparison BW
% showTwoImages(imagesBW, imagesBWFiltered, 'comparisonBW');

%% Grafico dell'immagine
% figure
% imageScaled = imresize(imagesBW, 0.1);
% mesh(imageScaled);

%% point 1
% TROVIAMO LE RUOTE MANUALMENTE

% find matric conic and profile
[C1, profile1] = findEllipses(imagesBW, 'wheel1');
% merge the conic and the image
imageWithWheels = showProfileOpt(imagesBW, profile1);

[C2, profile2] = findEllipses(imagesBW, 'wheel2');
imageWithWheels = showProfileOnImage(imageWithWheels, profile2, 0, 0);

% showTwoImages(imagesBW, imageWithWheels, 'conic as wheels')

%% point 2.1
% calculate bitanget
lines = bitanget(C1, C2);
tan1 = lines(:,1);
tan2 = lines(:,2);  % good one
tan3 = lines(:,3);  % good one
tan4 = lines(:,4);

% tangent points
v1 = intersection(C1, lines);
v1 = [v1(:,2) v1(:,1)]; % the first column represents the upper point in both cases
v2 = intersection(C2, lines);

% display the line profile in a binary image
tic
linesP = fromLinesToProfile(imagesBW, [tan2 tan3]);
toc
% add the conic profile
linesP = showProfileOnImage(linesP, profile1, 0, 0);
linesP = showProfileOnImage(linesP, profile2, 0, 0);

% show the image with tangent and conic
img = showProfileOnImage(imagesBW, linesP, 0,0);
linesFigure = figure('name', 'visualization');
imshow(img);


figure(linesFigure)
hold on
plot(v1(1,:), v1(2,:), 'or','MarkerSize',12, 'color', 'g');
plot(v2(1,:), v2(2,:), 'or','MarkerSize',12, 'color', 'g');
hold off

pause

% BACK TRANSFORMATION
figure(linesFigure)
hold on
line1 = cross(v1(:,1), v2(:,1));
line1 = line1/line1(3);
line2 = cross(v1(:,2), v2(:,2));
line2 = line2/line2(3);
vpoint1 = cross(line1, line2);
vpoint1 = vpoint1/vpoint1(3);   % first vanishing point

line3 = cross(v1(:,1), v1(:,2));
line3 = line3/line3(3);
line4 = cross(v2(:,1), v2(:,2));
line4 = line4/line4(3);
vpoint2 = cross(line3, line4);
vpoint2 = vpoint2/vpoint2(3);   % second vanishing point


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

% image of absolute conic at infinity 
Cinf = I*J' + J*I';
[U,S,V] = svd(Cinf);            % A = U*S*V'

% use T to transform S into 
% [1       ]
% [   1    ]
% [       1]
s11 = S(1,1);
s22 = S(2,2);
T = [ sqrt(s11)     0       0 ;...
        0       sqrt(s22)   0 ;...
        0           0       1];

bm = U*T;
Hr=inv(bm);
Hr=Hr/Hr(3,3);
% similarity matrix
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

% find vanishing point from the car license plate
[vps1, vps2, xT, yT] = findVPfromL(imagesBW);

figure(linesFigure)
hold on
plot(xT, yT, 'or','MarkerSize',12, 'color', 'g');
plot(vps1(1), vps1(2), 'or','MarkerSize',12, 'color', 'b');
plot(vps2(1), vps2(2), 'or','MarkerSize',12, 'color', 'y');
hold off

% camera matrix
syms fxI fyI u0I v0I;

KI = [  fxI     0       u0I;...
        0       fyI     v0I;...
        0       0       1];
% dual image absolute conic
wstar = KI*KI.';
% image of absolute conic
w = inv(wstar);

% constraints
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

% calibration matrix
K = [   fx  0   u0 ;...
        0   fy  v0 ;...
        0   0   1]
iac = inv(K*K.');       % w
iac = iac/iac(3,3);


%% point 2.3

% dimensione targa
% 520 mm x 110 mm;

% vanishing points
vpX = vpoint1;
vpY = vps1;
vpZ = vpoint2;
% image of absolute conic
w = iac;

position2d = figure('name', '2d position');
imshow(imagesBW);
hold on

% back transformation for each plane
HrYZ = backTransformation(vpY, vpZ, w);
HrXY = backTransformation(vpX, vpY, w);
HrXZ = backTransformation(vpX, vpZ, w);

% center of the system, the midpoint of lower side
center = crossRatioMid([xT(3) ; yT(3) ; 1], [xT(4) ; yT(4) ; 1], vpY);

% draw the axes
myline=[vpY.' ; center.' ; vpZ.' ; center.' ; vpX.'];
line(myline(:,1),myline(:,2),'LineWidth',1, 'color', 'c');
% plot the center
plot(center(1), center(2),'or','MarkerSize',12, 'color', 'r'); 
% plot the vanishing point
plot([vpX(1) vpY(1) vpZ(1)], [vpX(2) vpY(2) vpZ(2)], 'or','MarkerSize',12, 'color', 'r');   

% points of car lincense plate
p2 = [xT(3); yT(3); 1];
p1 = [xT(4); yT(4); 1];
plot(xT(3:4), yT(3:4),'or','MarkerSize',12, 'color', 'g');

% symmetric point
p2m = normalize((HrYZ*p2));
p1m = normalize((HrYZ*p1));

% center
cm = normalize((HrYZ*center));
% distance from center to p1 and p2 in YZ plane
dx1 = pdist([p1m(1:2).'; cm(1:2).'], 'euclidean');
dx2 = pdist([p2m(1:2).'; cm(1:2).'], 'euclidean');


pause
position3d = figure('name', '3d position');
scatter3(0, 0, 0, 'r', 'filled')
hold on
scatter3(0, dx1, 0, 'g', 'filled')
scatter3(0, -dx2, 0, 'g', 'filled')
pause

imagePoints = [];
points= [];
[pr, pc] = size(imagePoints);

str = 'yes';
while strcmp(str,'yes')
    [PA1, PA2, pa1, pa2] = plotSymP(imagesBW, center, HrXY, HrYZ, position2d, position3d, vpY);
    [pr, pc] = size(imagePoints);
    imagePoints(:, pc+1:pc+2) = [pa1 pa2];
    points(:, pc+1:pc+2) = [PA1 PA2];
%     imagePoints = [imagePoints PA1 PA2]
    [pr, pc] = size(imagePoints);
    pause
    prompt = 'Do you want select more points? Y/N [Y]: ';
    str = input(prompt,'s');
    if isempty(str)
        str = 'yes';
    elseif str == 'n' || str == 'N'
        if pc < 6
            str = 'yes';
            disp('find at least 3 pairs of points')
        else
            str = 'no';
        end
    else
        str = 'yes';
    end
end

%% point 2.4
p4 = [xT(2); yT(2); 1];
p3 = [xT(1); yT(1); 1];
pm = crossRatioMid([xT(1) ; yT(1) ; 1], [xT(2) ; yT(2) ; 1], vpY);

% projection on YZ plane
p3m = normalize(HrYZ*p3);       % P1
p4m = normalize(HrYZ*p4);       % P2
pmm = normalize(HrYZ*pm);       % midpoint
% projection on XY plane
pmx = normalize(HrXY*pm);       % midpoint
cmx = normalize(HrXY*center);   % center

% distance between each point to midpoint in YZ plane
d3 = pdist([p3m(1:2).'; pmm(1:2).'], 'euclidean');
d4 = pdist([p4m(1:2).'; pmm(1:2).'], 'euclidean');
d = (d3+d4)/2;  % media of distances
% distance between midpoint to center in YZ plane
dz = pdist([cm(1:2).'; pmm(1:2).'], 'euclidean');
% distance between midpoint to center in XY plane
dx = pdist([cmx(1:2).'; pmx(1:2).'], 'euclidean');
P1 = [0 -dx1 0 1].';
P2 = [0 dx1 0 1].';
P3 = [dx -d dz 1].';
P4 = [dx d dz 1].';

syms h11 h12 h13 h14 h21 h22 h23 h24 h31 h32 h33 h34;

H = [h11 h12 h13 h14;...
    h21 h22 h23 h24;...
    h31 h32 h33 h34;
    0 0 0 1];
P = [K zeros(3,1)];
eq1 = P*H*points(:,1) == imagePoints(:,1);
eq2 = P*H*points(:,4) == imagePoints(:,4);
eq3 = P*H*points(:,5) == imagePoints(:,5);
eq4 = P*H*points(:,6) == imagePoints(:,6);
%P*H*P1 == p1;

sol = solve([eq1 eq2 eq3 eq4], [h11 h12 h13 h14 h21 h22 h23 h24 h31 h32 h33 h34]);

Hcrtocm = [  double(sol.h11) double(sol.h12) double(sol.h13) double(sol.h14);...
        double(sol.h21) double(sol.h22) double(sol.h23) double(sol.h24);...
        double(sol.h31) double(sol.h32) double(sol.h33) double(sol.h34);...
        0               0               0               1];
    
Hcmtocr = [Hcrtocm(1:3,1:3).'       -(Hcrtocm(1:3,1:3).')*Hcrtocm(1:3,4);...
            zeros(1, 3)                     1];
        
focalPointCM = Hcmtocr*[0; 0; 0; 1];

figure(position3d)
% plot on 3D graph
hold on
scatter3(focalPointCM(1), focalPointCM(2), focalPointCM(3), 'r', 'filled')

%% test 
% clc
% [righe, colonne] = size(points);
% for i = 1:colonne
%     test1 = imagePoints(:,i);
%     test2 = P * Hcrtocm * points(:,i);
%     string = sprintf('\npoint number %d', i);
%     disp(string)
%     errore = abs(test1-test2)./test1
% end

%% plot a camera
% figure(position3d)
% hold on
% orientation1 = [Hcmtocr(1:3,1)/norm(Hcmtocr(1:3, 1))...
%                 Hcmtocr(1:3,2)/norm(Hcmtocr(1:3, 2))...
%                 Hcmtocr(1:3,3)];
% orientation = [1 0 0; 0 0 -1; 0 1 0];
%            
% cam = plotCamera('Location',Hcmtocr(1:3, 4).','Orientation',orientation,'Size',0.05);

%% remove object plotted from figure
% h = plot...
%     ...
% if exist('h', 'var')
%   delete(h)
% end