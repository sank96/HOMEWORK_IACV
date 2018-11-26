function [C profile] = findConic(img)
%FINDCONIC Starting from ad image (prefered with edge highlighted), it is asked
%user to insert 5 points in order to find the Conic.
%   FINDCONIC return the C matrix that describes the conic and halso a
%   binary image who represents the boundary of conicm

% display the input image in order to choose 5 points
figure('Name', 'find conic');
imshow(img);
hold on;
disp('click 5 points, then enter');
[x y]=getpts;
scatter(x,y,100,'filled');

A=[x.^2 x.*y y.^2 x y ones(size(x))];
% A is 5x6: we find the parameter vector (containing [a b c d e f]) as the
% right null space of A.
% This returns a vector cc such that A*cc=0.  Note that this expresses the
% fact that the conic passes through all the points we inserted.
N = null(A);

cc = N(:, 1);
[a b c d e f]=deal(cc(1),cc(2),cc(3),cc(4),cc(5),cc(6));
% here is the matrix of the conic
C=[a b/2 d/2; b/2 c e/2; d/2 e/2 f];
% Remark: since the right null space has dimension one,
% the system admits an infinite number of solutions
% however, these can be expressed as lambda * n, where n
% lambda \in R and n = null(A).
% thus, they all corresponds to the same conic.

[r c] = size(img);
im = zeros(r, c);
for i=1:r
    for j=1:c
        im(i,j)=[j i 1]*C*[j i 1]';
    end
end

im = im < 0;
profile = findEdges(im, 'binary');

close 'find conic';

end

