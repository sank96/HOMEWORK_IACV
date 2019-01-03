function [vpoint1, vpoint2, img, v1, v2] = findVPfromC(image)
%FINDVPFROMC Find vanishing points from a pair of coplanar conic
%   Detailed explanation goes here

[C1, profile1] = findConic(image, 'sensor1', false);
[C2, profile2] = findConic(image, 'sensor2', false);

% bitangenti
syms a b;
C1star = inv(C1);
C2star = inv(C2);
l = [a; b; 1];
A1 = l.' * C1star * l;
A2 = l.' * C2star * l;
sol = solve([A1 A2], [a b]);
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

imageProfile = fromLinesToProfile(image, [tan2 tan3]);
imageProfile = showProfileOnImage(imageProfile, profile1, 0, 0);
imageProfile = showProfileOnImage(imageProfile, profile2, 0, 0);
img = showProfileOnImage(image, imageProfile, 0,0);

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
end

