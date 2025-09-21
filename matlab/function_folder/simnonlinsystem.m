function [y, x] = simnonlinsystem(p, x0, u, Ts)


% 1) Parameters (fill in your measured values)
mc_mp   = p(1);    
mp_l   = p(2);
mp_l2_plus_Icom    = p(3);
cc   = p(4);
cp   = p(5);
k    = p(6);
g = 9.81;

% 2) Simulation setup
 
N    = length(u);

% 3) Pre‐allocate state & input
% State x = [theta; theta_dot; y; y_dot]
x     = zeros(4,N);
% initial condition
x(:,1) = x0;    % 0.1 rad initial displacement


% 4) Dynamics as a function handle
%  x = [θ; θ̇; y; ẏ],   F = k * u
f = @(x, F) [ ...
    x(2);  
    % θ̈
    ( - (mp_l*cos(x(1))) * ( F + mp_l*sin(x(1))*x(2)^2 - cc*x(4) ) ...
      - (mc_mp) * ( mp_l*g*sin(x(1)) + cp*x(2) ) ) ...
    / ( (mp_l2_plus_Icom)*(mc_mp) - (mp_l*cos(x(1)))^2 );  
    x(4);
    % ÿ
    (  (mp_l*cos(x(1))) * ( mp_l*g*sin(x(1)) + cp*x(2) ) ...
      + (mp_l2_plus_Icom) * ( F + mp_l*sin(x(1))*x(2)^2 - cc*x(4) ) ) ...
    / ( (mp_l2_plus_Icom)*(mc_mp) - (mp_l*cos(x(1)))^2 );  
];

% 5) RK4 integration loop
for i = 1:N-1
    F = k * u(i);
    k1 = f( x(:,i),     F );
    k2 = f( x(:,i) + 0.5*Ts*k1, F );
    k3 = f( x(:,i) + 0.5*Ts*k2, F );
    k4 = f( x(:,i) +     Ts*k3, F );

    x(:,i+1) = x(:,i) + (Ts/6)*(k1 + 2*k2 + 2*k3 + k4);
end
C = [1 0 0 0;0 0 1 0];

y = (C*x)'; 
end