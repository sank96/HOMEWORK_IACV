function s = findEllipses(image)
%FINDELLIPSES Usefull to find ellipses in an image
%   Detailed explanation goes here

% im_orig=image;
% 
% bw = edge(im_orig, 'canny');
bw = image;

s = regionprops(bw,{...
    'Centroid',...
    'MajorAxisLength',...
    'MinorAxisLength',...
    'Orientation'})

figure
imshow(bw,'InitialMagnification','fit')

t = linspace(0,2*pi,500);
conics = [];
hold on
for k = 1:length(s)
    a = s(k).MajorAxisLength/2;
    b = s(k).MinorAxisLength/2;
    Xc = s(k).Centroid(1);
    Yc = s(k).Centroid(2);
    phi = deg2rad(-s(k).Orientation);
    x = Xc + a*cos(t)*cos(phi) - b*sin(t)*sin(phi);
    y = Yc + a*cos(t)*sin(phi) + b*sin(t)*cos(phi);
    if(a > 20 && b > 20)    % 20 - 20
        plot(x,y,'r','Linewidth',2)
        conics = [conics s(k)];
    end      
end

pause
title('select the conic')
[x, y] = getpts;
plot(x, y, 'or', 'linewidth', 12)

A = [x, y];
t = linspace(0,2*pi,500);
mindistances = zeros(length(conics),1)
for k = 1:length(conics)
    a = conics(k).MajorAxisLength/2;
    b = conics(k).MinorAxisLength/2;
    Xc = conics(k).Centroid(1);
    Yc = conics(k).Centroid(2);
    phi = deg2rad(-conics(k).Orientation);
    x = Xc + a*cos(t)*cos(phi) - b*sin(t)*sin(phi);
    y = Yc + a*cos(t)*sin(phi) + b*sin(t)*cos(phi);
    B = [x.' y.'];
    distances = sqrt(sum(bsxfun(@minus, B, A).^2,2));
    closest = B(find(distances==min(distances)),:);
    mindistances(k) = [conics(k) closest]
end
%compute Euclidean distances:
% distances = sqrt(sum(bsxfun(@minus, B, A).^2,2));
%find the smallest distance and use that as an index into B:
% closest = B(find(distances==min(distances)),:);

end