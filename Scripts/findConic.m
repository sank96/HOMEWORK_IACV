function C = findConic(x, y)
%FINDCONIC From 5 points it returns the C matrix that describes the conic.
%   FINDCONIC From 5 points it returns the C matrix that describes the conic.


A=[x.^2 x.*y y.^2 x y ones(size(x))];
% A is 5x6: we find the parameter vector (containing [a b c d e f]) as the
% right null space of A.
% This returns a vector cc such that A*cc=0.  Note that this expresses the
% fact that the conic passes through all the points we inserted.
N = null(A);

cc = N(:, 1);
[a, b, c, d, e, f]=deal(cc(1),cc(2),cc(3),cc(4),cc(5),cc(6));
% here is the matrix of the conic
C=[a b/2 d/2; b/2 c e/2; d/2 e/2 f];
% Remark: since the right null space has dimension one,
% the system admits an infinite number of solutions
% however, these can be expressed as lambda * n, where n
% lambda \in R and n = null(A).
% thus, they all corresponds to the same conic.

end

