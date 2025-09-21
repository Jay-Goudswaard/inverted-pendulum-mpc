function H = hessian(psi)
%  p = size(theta); 
%  N = size(y, 1); 
%  l = size(C, 1);
%
% Function INPUT
% psi derivative of estimation error wrt theta (matrix of size l*N x p)
%
% Function OUTPUT
% H Hessian matrix (matrix of size p x p)

H = 2/size(psi,1) * (psi' * psi);

end