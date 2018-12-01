function testFun(x, y)
%TEST Summary of this function goes here
%   Detailed explanation goes here
xx = 0 ;
yy = -0.7;
t = pi/4; 
A = [cos(t) -sin(t) xx-cos(t)*xx+sin(t)*yy; sin(t) cos(t) yy-sin(t)*xx-cos(t)*yy; 0 0 1];
data = []; 
data = [x'; y'; ones(1,length(x))];
rotdat = A*data;
subplot(1,2,1) 
plot(x,y), axis ij, axis equal, axis([-2 2 -2 2]), 
grid on, 

title('George')
subplot(1,2,2) 
plot(rotdat(1,:), rotdat(2,:)), axis ij, axis equal, axis([-2 2 -2 2]) 
grid on
title('Rotated by 45\circ')
end

