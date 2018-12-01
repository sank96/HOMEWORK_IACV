function s = findEllipses(image)
%FINDELLIPSES Summary of this function goes here
%   Detailed explanation goes here

im_orig=image;

bw = edge(im_orig, 'canny');


s = regionprops(bw,{...
    'Centroid',...
    'MajorAxisLength',...
    'MinorAxisLength',...
    'Orientation'})

figure
imshow(bw,'InitialMagnification','fit')

t = linspace(0,2*pi,50);

hold on
for k = 1:length(s)
    a = s(k).MajorAxisLength/2;
    b = s(k).MinorAxisLength/2;
    Xc = s(k).Centroid(1);
    Yc = s(k).Centroid(2);
    phi = deg2rad(-s(k).Orientation);
    x = Xc + a*cos(t)*cos(phi) - b*sin(t)*sin(phi);
    y = Yc + a*cos(t)*sin(phi) + b*sin(t)*cos(phi);
    if(a > 10)
        plot(x,y,'r','Linewidth',2)
    end
        
end
hold off
end