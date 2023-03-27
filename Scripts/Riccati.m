function X=Riccati(A,G,Q)
%RICCATI  Solves an algebraic Riccati equation
%   X = Riccati(A,G,Q) solves the algebraic Riccati equation of the form:
%       A'*X + X*A - X*G*X + Q = 0, where X is symmetric.
% where A, G, and Q are given and X is the symmetric solution. 
% All terms are real nxn matrices and G and Q are positive semi-definite.
% A common use of this equation is to solve for the optimal feedback gain of a linear system, in which case
% G = B*R^-1*B' and K = R^-1*B'*X.
% 
% See http://en.wikipedia.org/wiki/Linear-quadratic_regulator for more detail. 
% Based on the solution method of http://dspace.mit.edu/handle/1721.1/1301.


n=size(A,1);

Z=[A -G
    -Q -A'];

[U1,S1]=schur(Z);
[U,S]=ordschur(U1,S1,'lhp');

X=U(n+1:end,1:n)*U(1:n,1:n)^-1;