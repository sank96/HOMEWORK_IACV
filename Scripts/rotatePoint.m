function pointRT = rotatePoint(point,alpha, image)
%ROTATION Summary of this function goes here
%   Detailed explanation goes here
 
centerIm = (size(image) ./2).';
centerIm = [centerIm(2); centerIm(1)];
pointT = point - centerIm;
th = alpha*pi/180 ;
R = [cos(th) -sin(th) ;sin(th) cos(th)] ;
%%rotate points
pointR = R*pointT ;

pointRT = pointR + centerIm;

end