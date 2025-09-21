% function [dAdp, dBdp, dx0dp] = getpartderivs(p, dimp, nx, h)
% % Function INPUT
% % p = parameters (1xnp)
% %
% % Function OUTPUT
% % dAdp partial A to p (tensor of size nx x nx x np)
% % dBdp partial B to p (tensor of size nx x nu x np)
% % dx0dp partial x0 to p (tensor of size nx x 1 x np)
% 
% syms mc mp l cc cp Icom 
% g = 9.81;
% 
% % Common denominator
% Delta = mc*mp*l^2 + Icom*(mp + mc);
% 
% % State-space A matrix
% A = sym(zeros(4, 4));
% A(1,2) = 1;
% A(2,1) = -mp * l * g * (mc + mp) / Delta;
% A(2,2) = -cp * (mc + mp) / Delta;
% A(2,4) = cc * mp * l / Delta;
% A(3,4) = 1;
% A(4,1) = mp^2 * l^2 * g / Delta;
% A(4,2) = cp * mp * l / Delta;
% A(4,4) = -cc * (mp * l^2 + Icom) / Delta;
% 
% % Input B matrix
% B = sym(zeros(4,1));
% B(2) = (mp * l^2 + Icom) / Delta;
% B(4) = -mp * l / Delta;
% 
% 
% % Param vector
% param_syms = [mc, mp, l, cc, cp, Icom];
% np = length(param_syms);
% nx = 4;
% nu = 1;
% 
% % Preallocate gradient tensors
% dAdp = zeros(nx, nx, np);
% dBdp = zeros(nx, nu, np);
% 
% % Compute symbolic gradients
% for i = 1:np
%     dA_sym = simplify(diff(A, param_syms(i)));
%     dB_sym = simplify(diff(B, param_syms(i)));
% 
%     % Substitute numerical values
%     for r = 1:nx
%         for c = 1:nx
%             % size(param_syms)
%             % size(p)
%             dAdp(r, c, i) = double(subs(dA_sym(r, c), param_syms, p'));
%         end
%         for c = 1:nu
%             dBdp(r, c, i) = double(subs(dB_sym(r, c), param_syms, p'));
%         end
%     end
% end
% 
% % Apply sampling time (Euler method)
% dAdp = h * dAdp;
% dBdp = h * dBdp;
% 
% 
% % % Initialize derivatives of A matrix (using forward-Euler approx):
% % dAdp(:,:,1) = [p(4)/(p(1)^2)   0                        0;
% %                0              0                         0;
% %                0              0                        0];
% % 
% % dAdp(:,:,2) = [p(4)/(p(2)^2)   -p(3)/(p(2)^2)   -p(5)*9.81/(p(2)^2);
% %               -p(4)/(p(2)^2)    p(3)/(p(2)^2)  p(5)*9.81/(p(2)^2);
% %                0              0                        0];
% % 
% % dAdp(:,:,3) = [0       1/p(2)    0;
% %                0     -1/p(2)    0;
% %                0        0       0];
% % 
% % dAdp(:,:,4) = [-(p(1)+p(2))/(p(1)*p(2))   0     0;
% %                1/p(2)                0     0;
% %                0                    0     0];
% % 
% % dAdp(:,:,5) = [0          0                      9.81/p(2);
% %                0          0                     -9.81/p(2);
% %                0          0                      0];
% % 
% % dAdp(:,:,6) = zeros(nx);
% % dAdp = h * dAdp;
% % 
% % % Initialize derivatives of B matrix:
% % dBdp(:,1,1) = [-p(6)/(p(1)^2); 0; 0];
% % dBdp(:,1,2) = [-p(6)/(p(2)^2); p(6)/(p(2)^2); 0];
% % dBdp(:,1,3) = [0; 0; 0];
% % dBdp(:,1,4) = [0; 0; 0];
% % dBdp(:,1,5) = [0; 0; 0];
% % dBdp(:,1,6) = [(p(1)+p(2))/(p(1)*p(2)); -1/p(2); 0];
% % dBdp = h * dBdp;
% 
% % Zero initial condition sensitivity
% dx0dp = zeros(nx, 1, np);
% 
% end
% 

classdef getpartderivs
    properties
        param_syms   % symbolic parameter vector
        A_sym        % symbolic A matrix
        B_sym        % symbolic B matrix
        dA_syms      % symbolic gradients of A wrt params
        dB_syms      % symbolic gradients of B wrt params
        nx = 4
        nu = 1
        np = 6
        g = 9.81
    end

    methods
        function obj = getpartderivs()
            % Define symbols
            % syms mc mp l cc cp Icom k real
            % p = [mc, mp, l, cc, cp, Icom, k];
            syms mc_mp mp_l mp_l2_plus_Icom cc cp k real
            p = [mc_mp, mp_l, mp_l2_plus_Icom, cc, cp, k];
            obj.param_syms = p;

            % Common denominator
            Delta = mc_mp*mp_l2_plus_Icom - mp_l^2 ;%mc*mp*l^2 + Icom*(mp + mc);
            
            % State-space A matrix
            A = sym(zeros(4, 4));
            A(1,2) = 1;
            A(2,1) = -mp_l * obj.g * (mc_mp) / Delta;
            A(2,2) = -cp * (mc_mp) / Delta;
            A(2,4) = cc * mp_l / Delta;
            A(3,4) = 1;
            A(4,1) = (mp_l)^2 * obj.g / Delta;
            A(4,2) = cp * mp_l / Delta;
            A(4,4) = -cc * (mp_l2_plus_Icom) / Delta;
            
            % Input B matrix
            B = sym(zeros(4,1));
            B(2) = -k * mp_l / Delta;
            B(4) = k*(mp_l2_plus_Icom) / Delta;

            obj.A_sym = A;
            obj.B_sym = B;
            
            % Fix: initialize as symbolic arrays
            obj.dA_syms = sym(zeros(obj.nx, obj.nx, obj.np));
            obj.dB_syms = sym(zeros(obj.nx, obj.nu, obj.np));

            % Compute symbolic derivatives
            for i = 1:obj.np
                obj.dA_syms(:,:,i) = simplify(diff(A, p(i)));
                obj.dB_syms(:,:,i) = simplify(diff(B, p(i)));
            end
        end

        function [dAdp, dBdp, dx0dp] = evaluate(obj, p_val, h)
            dAdp = zeros(obj.nx, obj.nx, obj.np);
            dBdp = zeros(obj.nx, obj.nu, obj.np);

            for i = 1:obj.np
                dA_num = double(subs(obj.dA_syms(:,:,i), obj.param_syms, p_val(:)'));
                dB_num = double(subs(obj.dB_syms(:,:,i), obj.param_syms, p_val(:)'));
                dAdp(:,:,i) = h * dA_num;
                dBdp(:,:,i) = h * dB_num;
            end

            dx0dp = zeros(obj.nx, 1, obj.np);
        end
    end
end