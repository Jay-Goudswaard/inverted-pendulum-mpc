function J = jacobian(psi,E)
%  p = size(theta); 
%  N = size(y, 1); 
%  l = size(C, 1);
%
% Function INPUT
% psi derivative of estimation error wrt theta (matrix of size l*N x p)
% E estimation error vector (vector of size l*N x one)
%
% Function OUTPUT
% J Jacobian vector (vector of size p x one)

J = 2/size(psi,1) * psi' * E;


end