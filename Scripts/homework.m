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
[C1, profile1] = findConic(imagesBW, 'wheel1');
% merge the conic and the image
imageWithWheels = showProfileOpt(imagesBW, profile1);

[C2, profile2] = findConic(imagesBW, 'wheel2');
imageWithWheels = showProfileOnImage(imageWithWheels, profile2, 0, 0);

% showTwoImages(imagesBW, imageWithWheels, 'conic as wheels')
% find automatically ellipses
% s = findEllipses(imagesBW);

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

% dimensione targa
% 520 mm � 110 mm;

vpX = vpoint1;
vpY = vps1;
vpZ = vpoint2;
w = iac;

close all
position2d = figure('name', '2d position');
imshow(imagesBW);
hold on
% plot(xT(3:4), yT(3:4),'or','MarkerSize',12, 'color', 'g');

HrYZ = backTransformation(vpY, vpZ, w);
HrXY = backTransformation(vpX, vpY, w);
HrXZ = backTransformation(vpX, vpZ, w);

% p = [ HrYZ * [xT(3) ; yT(3) ; 1] ...
%     HrYZ * [xT(4) ; yT(4) ; 1]];
% p = [ p(:,1)/p(3,1) p(:,2)/p(3,2)];
% cp = (p(:,1)+p(:,2))/2;
% center = inv(HrYZ) * cp;

center = crossRatioMid([xT(3) ; yT(3) ; 1], [xT(4) ; yT(4) ; 1], vpY);
% punto medio della targa in basso

myline=[vpY.' ; center.' ; vpZ.' ; center.' ; vpX.'];
line(myline(:,1),myline(:,2),'LineWidth',1, 'color', 'c');
plot(center(1), center(2),'or','MarkerSize',12, 'color', 'r');
plot([vpX(1) vpY(1) vpZ(1)], [vpX(2) vpY(2) vpZ(2)], 'or','MarkerSize',12, 'color', 'r');


p2 = [xT(3); yT(3); 1];
p1 = [xT(4); yT(4); 1];
p3 = [xT(1); yT(1); 1];
p4 = [xT(2); yT(2); 1];
plot(xT(3:4), yT(3:4),'or','MarkerSize',12, 'color', 'g');
pm = crossRatioMid([xT(1) ; yT(1) ; 1], [xT(2) ; yT(2) ; 1], vpY);
% plot(pm(1), pm(2), 'or', 'markersize', 12, 'color', 'b');

pmm = normalize((HrYZ*pm));
p4m = normalize((HrYZ*p4));
p3m = normalize((HrYZ*p3));
p2m = normalize((HrYZ*p2));
p1m = normalize((HrYZ*p1));
cm = normalize((HrYZ*center));
dx1 = pdist([p1m(1:2).'; cm(1:2).'], 'euclidean');
dx2 = pdist([p2m(1:2).'; cm(1:2).'], 'euclidean');
dz = pdist([cm(1:2).'; pmm(1:2).'], 'euclidean');
dx3 = pdist([p3m(1:2).'; pmm(1:2).'], 'euclidean');
dx4 = pdist([p4m(1:2).'; pmm(1:2).'], 'euclidean');


pause
position3d = figure('name', '3d position');
scatter3(0, 0, 0, 'r', 'filled')
hold on
scatter3(0, dx1, 0, 'g', 'filled')
scatter3(0, -dx2, 0, 'g', 'filled')
% scatter3(0, 0, dz, 'b', 'filled')
% scatter3(0, dx3, dz, 'g', 'filled')
% scatter3(0, -dx4, dz, 'g', 'filled')
pause

str = 'yes';
while strcmp(str,'yes')
    plotSymP(imagesBW, center, HrXY, HrYZ, position2d, position3d, vpY)
    pause
    prompt = 'Do you want select more points? Y/N [Y]: ';
    str = input(prompt,'s');
    if isempty(str)
        str = 'yes';
    elseif str == 'n' || str == 'N'
        str = 'no';
    else
        str = 'yes';
    end
end

%% point 2.4
Om = K(:,3);

figure(position2d)
hold on
plot(Om(1), Om(2), 'or', 'markersize', 12, 'color', 'r')

a = pdist([vpZ(1:2).'; vpX(1:2).'], 'euclidean');
b = pdist([Om(1:2).'; vpZ(1:2).'], 'euclidean');
c = pdist([Om(1:2).'; vpX(1:2).'], 'euclidean');

focalDistance = sqrt((a*a - c*c - b*b)/2);
%%
[Mit1, omegaInf, N] = svd(w);
T1 = [ sqrt(omegaInf(1,1))     0                 0 ;...
        0                sqrt(omegaInf(2,2))     0 ;...
        0                      0            sqrt(omegaInf(3,3))];

Mit = Mit1 * T1;
Mi = Mit.';
M = inv(Mi);

Rcw1= inv(K)*M;
Rcw = [ Rcw1(:,1)/norm(Rcw1(:,1)) ...
        Rcw1(:,2)/norm(Rcw1(:,2)) ...
        Rcw1(:,3)/norm(Rcw1(:,3))];


Rwc = inv(Rcw);
p1w = [0; dx1; 0];
p2w = [0; -dx2; 0];
p1c = Rwc * p1w
p2c = Rwc * p2w
pmc = (p1c+p2c)



%%
figure(position2d)
hold on
p = [v1(1,1); v1(2,1); 1];
plot(p(1), p(2), 'or','MarkerSize',12, 'color', 'g');

x1 = normalize(cross(vpX, center));
x2 = normalize(cross(vpX, p));
y1 = normalize(cross(vpY, center));
y2 = normalize(cross(vpY, p));
z1 = normalize(cross(vpZ, center));
z2 = normalize(cross(vpZ, p));

px1y2 = normalize(cross(x1, y2));
px2y1 = normalize(cross(x2, y1));
py1z2 = normalize(cross(y1, z2));
py2z1 = normalize(cross(y2, z1));
pz1x2 = normalize(cross(z1, x2));
pz2x1 = normalize(cross(z2, x1));

plot(px1y2(1), px1y2(2), 'or','MarkerSize',12, 'color', 'b');
plot(px2y1(1), px2y1(2), 'or','MarkerSize',12, 'color', 'b');

plot(py1z2(1), py1z2(2), 'or','MarkerSize',12, 'color', 'r');
plot(py2z1(1), py2z1(2), 'or','MarkerSize',12, 'color', 'r');

plot(pz1x2(1), pz1x2(2), 'or','MarkerSize',12, 'color', 'g');
plot(pz2x1(1), pz2x1(2), 'or','MarkerSize',12, 'color', 'g');

line2 = [center.'; py1z2.' ; p.'; py2z1.'; center.'];
line(line2(:,1), line2(:,2), 'LineWidth', 2, 'color', 'r');
line3 = [center.'; pz1x2.' ; p.'; pz2x1.'; center.'];
line(line3(:,1), line3(:,2), 'LineWidth', 2, 'color', 'g');
line1 = [center.'; px1y2.' ; p.'; px2y1.'; center.'];
line(line1(:,1), line1(:,2), 'LineWidth', 2, 'color', 'b');

pt = HrXY * p;
pt = pt/pt(3);
ct = HrXY * center;
ct = ct/ct(3);
p12t = HrXY * px1y2;
p12t = p12t/p12t(3);
p21t = HrXY * px2y1;
p21t = p21t/p21t(3);

dpc = pdist([pt(1:2).';ct(1:2).'], 'euclidean')
d12c = pdist([p12t(1:2).';ct(1:2).'], 'euclidean')
d21c = pdist([p21t(1:2).';ct(1:2).'], 'euclidean')
sqrt(d12c*d12c + d21c*d21c)
