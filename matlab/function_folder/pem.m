function [Abar,Bbar,C,D,x0, J, H, p] = pem(p,A0,B0,C0,D0,x00,y,u,lambda,maxiter,Ts)
% Function INPUT
% theta  Paramter vector (size: depends on your mapping choice)
% A0 Initial guess for system matrix A (matrix of size n x n)
% B0 Initial guess for system matrix B (matrix of size n x m)
% C0 Initial guess for system matrix C (matrix of size l x n)
% D0 Initial guess for system matrix D (matrix of size l x m)
% x00 Initial guess for initial state (vector of size n x one)
% u System input (matrix of size N x m)
% y System output (matrix of size N x l)
% lambda regularization parameter (scalar)
% maxiter Maximum number of iterations (scalar)
%
%
% Function OUTPUT
% Abar Estimate of system matrix A (matrix of size n x n)
% Bbar Estimate of system matrix B (matrix of size n x m)
% C Estimate of system matrix C (matrix of size l x n)
% D Estimate of system matrix D (matrix of size l x m)
% x0 Estimate of initial state (vector of size n x one)

nx = size(A0,1);
ny = size(y,2);

figure;
cost_values = []; % to store cost function values
iterations = [];  % to store corresponding iterations
h = animatedline('Marker','.');
xlabel('Iteration');
ylabel('Cost Function');
title('Live Update of Cost Function');
ylim('padded'); xlim([0 maxiter])
grid on;
hold on;

q0 = log(p);
q = q0;
% weights for output scaling
w_rail = 1;
w_pend = 1;
weights = diag([w_pend, w_rail]);

% Gauss-Newton
% initialize
converged = false;
maxiter_reached = false;
iter = 1;
old_cost = 1e15;
PD = getpartderivs();
while ~converged && ~maxiter_reached 
    %obtain derivatives
    [dAdp, dBdp, dx0dp] = PD.evaluate(p, Ts);
    % chain-rule: build derivatives wrt q instead of p
    dAdq = zeros(size(dAdp));
    dBdq = zeros(size(dBdp));
    for i = 1:length(p)
      dAdq(:,:,i) = dAdp(:,:,i) * p(i);
      dBdq(:,:,i) = dBdp(:,:,i) * p(i);
    end
    % Get state space matrices from theta
    
    p = exp(q);
    [A, B, C, D, K, x0] = theta2matrices(p, Ts);

    % Compute Error
    [y_hat, x_hat] = simsystem(A, B, C, D, K, x0, u, y);
    x_hat = x_hat';
    for i = 1:length(y)
        E(ny*i-ny+1:ny*i,1) = weights*(y(i,:)' - y_hat(i,:)');
    end
    cost = E'*E;

    % Compute psi
    for np = 1:length(p)
        % dxdp(1:nx,np) = [dx0dp(:,:,np)];
        dxdp_piece(1:nx,np) = [dx0dp(:,:,np)];
        for k = 2:length(y)
        %     dxdp(nx*k-nx+1:nx*k,np) = [dAdp(:,:,np)*x_hat(:,k-1) + dBdp(:,:,np)*u(k-1) + (A-K*C)*dxdp(nx*(k-1)-nx+1:nx*(k-1),np)];
        dxdp_piece(nx*k-nx+1:nx*k,np) = [dAdq(:,:,np)*x_hat(:,k-1) + dBdq(:,:,np)*u(k-1) + (A-K*C)*dxdp_piece(nx*(k-1)-nx+1:nx*(k-1),np)];
        end
    end
    for entry = 1:length(y)
        % psi(ny*entry-ny+1:ny*entry,:) = weights'*weights * -C*dxdp(nx*entry-nx+1:nx*entry,:);
        psi(ny*entry-ny+1:ny*entry,:) = weights'*weights * -C*dxdp_piece(nx*entry-nx+1:nx*entry,:);
    end

    % Get jacobian
    J = jacobian(psi, E);
    
    % Get hessian: with psi
    H = hessian(psi);

    % theta_new = theta - hessian*jacobian
    % p_new = p - ((H + lambda * eye(size(H)))\J);
    q_new = q - ( (H + lambda*eye(size(H))) \ J );
    q = q_new;
    p_new = exp(q);  
    %Check convergence
    if (old_cost - cost < 1e-5) && (iter > round(0.1*maxiter))
        converged = true;
        fprintf("Converged after %i iterations \n",iter)
    elseif iter == maxiter 
        maxiter_reached = true; 
        warning('Maximum iterations reached'); 
    end
    p = p_new; 
    
    if mod(iter,10) == 0
        fprintf("At iteration i = %i \n",iter)
    end
    addpoints(h, iter, cost)
    drawnow limitrate;
    iter = iter + 1;
    old_cost = cost;
end

[Abar, Bbar, C, D, ~, x0] = theta2matrices(p, Ts);

end