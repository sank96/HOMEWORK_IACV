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

mainPoints = findCorner(imagesBW, 'find corner');
mainPoints = [mainPoints.' ; ones(1,length(mainPoints(:,1)))];
figure('name', 'main features'), imshow(imageWithWheels)
hold on
plot(mainPoints(1,:), mainPoints(2,:), 'g+', 'LineWidth', 4, 'color', 'r')


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

disp('press key to continue'),pause

% BACK TRANSFORMATION
figure(linesFigure)
hold on
line1 = cross(v1(:,1), v2(:,1));
line1 = line1/line1(3); % normalization of points and lines to reppresent them in homogeneous coordinate
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
% system of equations
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

diameter =  pdist([v1bt(1,1) v1bt(2,1);v1bt(1,2) v1bt(2,2)],'euclidean');
distancewtow = pdist([v1bt(1,1) v1bt(1,2);v2bt(1,1) v2bt(1,2)],'euclidean');
diameter1 = pdist([v2bt(1,1) v2bt(2,1);v2bt(1,2) v2bt(2,2)],'euclidean');

ratio = diameter/distancewtow
ratio1 = diameter1/distancewtow
% compute the ratio between both wheels to have a proof

%% point 2.2
% find the image of absolute conic

% find vanishing point from the car license plate
% [vps1, vps2, xT, yT] = findVPfromL(imagesBW);
[vps1, vps2, xT, yT] = findVPfromP(imagesBW, mainPoints);
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
pause(0.1)
imshow(imagesBW);
hold on

% back transformation for each plane
HrYZ = backTransformation(vpY, vpZ, w);
HrXY = backTransformation(vpX, vpY, w);
HrXZ = backTransformation(vpX, vpZ, w);

% center of the system, the midpoint of lower side
% center = crossRatioMid([xT(3) ; yT(3) ; 1], [xT(4) ; yT(4) ; 1], vpY);
center = crossRatioMid(mainPoints(:,3), mainPoints(:,4), vpY);

% draw the axes
myline=[vpY.' ; center.' ; vpZ.' ; center.' ; vpX.'];
line(myline(:,1),myline(:,2),'LineWidth',1, 'color', 'c');
% plot the center
plot(center(1), center(2),'or','MarkerSize',12, 'color', 'r');
% plot the vanishing point
plot([vpX(1) vpY(1) vpZ(1)], [vpX(2) vpY(2) vpZ(2)], 'or','MarkerSize',12, 'color', 'r');

disp('press key to continue'),pause

% 3D plot
position3d = figure('name', '3d position');
scatter3(0, 0, 0, 'r', 'filled')
xlabel('X axis'); ylabel('Y axis'); zlabel('Z axis');
hold on

disp('press key to continue'),pause

imagePoints = mainPoints;
points= [];
midpoints = [];
imageMid = [];
for i=1:2:length(mainPoints(1,:))
    [P1, P2, M, m] = plotSyms(imagePoints(:,i), imagePoints(:,i+1), center, HrXY, HrYZ, position2d, position3d, vpY);
    %findSymP(imagesBW, center, HrXY, HrYZ, position2d, position3d, vpY);
    points(:, i:i+1) = [P1 P2];
    midpoints(:, ceil(i)) = M;
    imageMid(:, ceil(i)) = m;
end


%% OLD version
% % this was written when I didn't find the main points, so I took them manually
% imagePoints = [];
% points= [];
% midpoints = [];
% imageMid = [];
% [pr, pc] = size(imagePoints);
%
% str = 'yes';
% while strcmp(str,'yes')
%     [PA1, PA2, pa1, pa2, M, m] = findSymP(imagesBW, center, HrXY, HrYZ, position2d, position3d, vpY);
%     [pr, pc] = size(imagePoints);
%     imagePoints(:, pc+1:pc+2) = [pa1 pa2];
%     points(:, pc+1:pc+2) = [PA1 PA2];
%     [mr, mc] = size(midpoints);
%     midpoints(:, mc+1) = M;
%     imageMid(:, mc+1) = m;
%
% %     imagePoints = [imagePoints PA1 PA2]
%     [pr, pc] = size(imagePoints);
%     disp('press key to continue'),pause
%     prompt = 'Do you want select more points? Y/N [Y]: ';
%     str = input(prompt,'s');
%     if isempty(str)
%         str = 'yes';
%     elseif str == 'n' || str == 'N'
%         if pc < 6
%             str = 'yes';
%             disp('find at least 3 pairs of points')
%         else
%             str = 'no';
%         end
%     else
%         str = 'yes';
%     end
% end

%% point 2.4
syms h11 h12 h13 h14 h21 h22 h23 h24 h31 h32 h33 h34;

H = [h11 h12 h13 h14;...
    h21 h22 h23 h24;...
    h31 h32 h33 h34;
    0 0 0 1];
P = [K zeros(3,1)];
eq1 = P*H*points(:,5) == imagePoints(:,5);
eq2 = P*H*points(:,7) == imagePoints(:,7);
eq3 = P*H*points(:,9) == imagePoints(:,9);
eq4 = P*H*points(:,13) == imagePoints(:,13);

sol = solve([eq1 eq2 eq3 eq4], [h11 h12 h13 h14 h21 h22 h23 h24 h31 h32 h33 h34]);

Hcrtocm = [  double(sol.h11) double(sol.h12) double(sol.h13) double(sol.h14);...
        double(sol.h21) double(sol.h22) double(sol.h23) double(sol.h24);...
        double(sol.h31) double(sol.h32) double(sol.h33) double(sol.h34);...
        0               0               0               1];

Hcmtocr = [Hcrtocm(1:3,1:3).'       -(Hcrtocm(1:3,1:3).')*Hcrtocm(1:3,4);...
            zeros(1, 3)                     1];

HH = Hcmtocr;
focalPointCM = Hcmtocr*[0; 0; 0; 1];

ttt = 0.01;
figure(position3d)
% plot on 3D graph
hold on
focalPointCM = -focalPointCM;
h = scatter3(focalPointCM(1), focalPointCM(2), focalPointCM(3), 'r', 'filled');
t = text(focalPointCM(1)+ttt, focalPointCM(2)+ttt, focalPointCM(3)+ttt, 'h');

return

%% CODICE AGGIUNTO
% %% without syms
% NN = imagePoints/points;
% MM = P\NN;
% MM(4,4) = 1;
% RR = MM(1:3, 1:3);
% % RR = [RR(:,1)/norm(RR(:,1))     RR(:,2)/norm(RR(:,2))       RR(:,3)/norm(RR(:,3))];
% HH1 = [RR.'       -(RR.')*MM(1:3,4);...
%             zeros(1, 3)                     1];
%
% focalPointCM1 = HH1*[0; 0; 0; 1]
% figure(position3d)
% % plot on 3D graph
% hold on
% focalPointCM1 = - focalPointCM1;
% h1 = scatter3(focalPointCM1(1), focalPointCM1(2), focalPointCM1(3), 'c', 'filled');
% t1 = text(focalPointCM1(1)+ttt, focalPointCM1(2)+ttt, focalPointCM1(3)+ttt, 'h1');
% %% calculate H with mid point
% p = [midpoints(1,:) ; midpoints(3:4,:)];
% NN = imageMid/p;
% MM = inv(K) * NN;
% tt = cross(MM(:,1), MM(:,2));
% RR = [MM(:,1)/norm(MM(:,1)) tt/norm(tt) MM(:,2)/norm(MM(:,2))];
%
% HHh1 = [RR ; 0 0 0];
% HHh1 = [HHh1 [MM(:,3) ; 1]];
%
% HH2 = [RR.' -(RR.')*MM(:,3);...
%     0 0 0 1]
% focalPointCM2 = HH2*[0; 0; 0; 1]
%
% figure(position3d)
% % plot on 3D graph
% hold on
% focalPointCM2 = -focalPointCM2;
% h2 = scatter3(focalPointCM2(1), focalPointCM2(2), focalPointCM2(3), 'c', 'filled');
% t2 = text(focalPointCM2(1)+ttt, focalPointCM2(2)+ttt, focalPointCM2(3)+ttt, 'h2');
%
% %% test
% clc
% [righe, colonne] = size(points);
% for i = 1:colonne
%     test1 = imagePoints(:,i);
%     test2 = normalize(P * HH1 * points(:,i));
%     string = sprintf('\npoint number %d', i);
%     disp(string)
%     errore = abs(test1-test2) ./ test1
% end
%
% %% plot a camera
% figure(position3d)
% hold on
% % orientation = [Hcrtocm(1:3,1)/norm(Hcrtocm(1:3, 1))...
% %                 Hcrtocm(1:3,2)/norm(Hcrtocm(1:3, 2))...
% %                 Hcrtocm(1:3,3)/norm(Hcrtocm(1:3, 3))];
% theta = 45;
% R2 = [         0.7071   -0.7071         0;...
%                0.7071    0.7071         0;...
%                0         0              1.0000];
%
% R1 = [1 0 0; 0 0 -1; 0 1 0];
% orientation = R1*R2;
% cam = plotCamera('Location',focalPointCM(1:3),'Orientation',orientation,'Size',0.05);
%
% %% remove object plotted from figure
% % h = plot...
% %     ...
% getname = @(x) inputname(1);
%
% if exist('cam', 'var')
%   delete(cam)
%   delete(t)
% end
