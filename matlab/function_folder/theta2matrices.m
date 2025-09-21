function [Abar,Bbar,C,D,K,x0] = theta2matrices(p,h)
%%
% Function INPUT
% theta     Parameter vector (vector of size: according to the realization)
%           Jrw, J, c1, c2, (full) ml, k
%
%
% Function OUTPUT
% Abar      System matrix A (matrix of size n x n)
% Bbar      System matrix B (matrix of size n x m)
% C         System matrix C (matrix of size l x n)
% D         System matrix D (matrix of size l x m)
% x0        Initial state (vector of size n x one)
% Parameters from vector p
% mc   = p(1);
% mp   = p(2);
% l    = p(3);
% cc   = p(4);
% cp   = p(5);
% Icom = p(6);
% k    = p(7);
mc_mp   = p(1);
mp_l   = p(2);
mp_l2_plus_Icom    = p(3);
cc   = p(4);
cp   = p(5);
k    = p(6);
g = 9.81;

% Common denominator
Delta = mc_mp*mp_l2_plus_Icom - mp_l^2 ;%mc*mp*l^2 + Icom*(mp + mc);

% State-space A matrix
A = zeros(4, 4);
A(1,2) = 1;
A(2,1) = -mp_l * g * (mc_mp) / Delta;
A(2,2) = -cp * (mc_mp) / Delta;
A(2,4) = cc * mp_l / Delta;
A(3,4) = 1;
A(4,1) = (mp_l)^2 * g / Delta;
A(4,2) = cp * mp_l / Delta;
A(4,4) = -cc * (mp_l2_plus_Icom) / Delta;

% Input B matrix
B = zeros(4,1);
B(2) = -k * mp_l / Delta;
B(4) = k*(mp_l2_plus_Icom) / Delta;
C=[1 0 0 0;0 0 1 0];

D=[0; 0]; 
x0=[0; 0; 0; 0];

sysd = c2d(ss(A,B,C,D),h,'zoh');

Q = diag([1e-4, 1e-4, 1e-4, 1e-4]);
R = diag([1e-2, 1e-1]);

Abar = sysd.A;
Bbar = sysd.B;

[~,~,K] = dare(Abar', C', Q, R);
K = K';%zeros(size(K'));

end

