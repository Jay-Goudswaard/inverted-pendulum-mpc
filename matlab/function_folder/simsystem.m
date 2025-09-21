function [y, x] = simsystem(A, B, C, D, K, x0, u, y_inp)
% Instructions:
% Simulating a linear dynamic system given input u, matrices A,B,C,D ,and
% initial condition x(0)
%
%  n = size(A, 1); 
%  m = size(B, 2); 
%  l = size(C, 1);
%  N = size(u, 1); 
%
% Function INPUT
% A system matrix (matrix of size n x n)
% B system matrix (matrix of size n x m)
% C system matrix (matrix of size l x n)
% D system matrix (matrix of size l x m)
% x0 initial state (vector of size n x one)
% u system input (matrix of size N x m)
%
% Function OUTPUT
% x state of system (matrix of size N x n)
% y system output (matrix of size N x l)

x(:,1) = x0;

for i = 2:length(u)
    x(:,i) = (A-K*C)*x(:,i-1) + B*u(i-1,:) + K*y_inp(i-1,:)';
end

y = (C*x + D*u')'; 
x = x';
end