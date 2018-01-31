classdef Fast_MPC2
    properties
        Q
        R
        S
        q
        r
        Qf
        qf
        x_min
        x_max
        u_min
        u_max
        T
        x0
        A
        B
        w
        x_final
        x_init
    end
    methods
        function cs = Fast_MPC2(Q,R,S,Qf,q,r,qf,xmin,xmax,umin,umax,T,x0,...
                A,B,w,xf,x_init)
            if nargin > 1
                cs.Q = Q;
                cs.R = R;
                cs.S = S;
                cs.Qf = Qf;
                cs.q = q;
                cs.r = r;
                cs.qf = qf;
                cs.x_min = xmin;
                cs.x_max = xmax;
                cs.u_min = umin;
                cs.u_max = umax;
                cs.T = T;
                cs.x0 = x0;
                cs.A = A;
                cs.B = B;
                cs.w = w;
                cs.x_final = xf;
                cs.x_init = x_init;
            end
        end
        function [H,g] = objective_function(obj)
            [H,g] = fast_mpc_objective(obj);
        end
        function [P,h] = inequality_const(obj)
            [P,h] = fast_mpc_ineq_const(obj);
        end
        function [C,b] = equality_const(obj)
            [C,b] = fast_mpc_eq_const(obj);
        end
        function [z_init] = initialize(obj)
            z_init = fast_mpc_init(obj);
        end
        function [J,A_eq,b_eq] = fomulate_mpc(obj,k)
           [P,h] =  inequality_const(obj);
           [H,g] = objective_function(obj);
           J = @(z)(z'*H*z + g'*z + k*(-sum(log(h - P*z))));
           
           [A_eq,b_eq] = equality_const(obj);
           
        end
        function [x_opt] = matlab_solve(obj)
            k = 1;
            mu = 1/10;
            z_init = initialize(obj);
            options = optimoptions(@fmincon,'Algorithm','interior-point','Display','off');
            while k*length(z_init) >= 10e-3
                [J,A_eq,b_eq] = fomulate_mpc(obj,k);
                x_opt = fmincon(J,z_init,[],[],A_eq,b_eq,[],[],[],options);
                z_init = x_opt;
                k = mu*k;
            end
        end
        function [x_opt] = mpc_solve_check(obj,k_min,k_max)
            k = linspace(k_max, k_min,5);
            [z] = initialize(obj);
            [H,g] = objective_function(obj);
            [P,h] = inequality_const(obj);
            [C,b] = equality_const(obj);
            nw = [];
            for i=1:length(k)
                x_opt = inf_newton_solver(H,g,P,h,C,b,k(i),z,nw);
                z = x_opt;
            end
        end
        function [x_opt] = mpc_solve_full(obj)
            k = 1;
            mu = 1/10;
            [H,g] = objective_function(obj);
            [P,h] = inequality_const(obj);
            [C,b] = equality_const(obj);
            [z] = initialize(obj);
            nw = [];
            while k*length(z) >= 10e-3
%                 fprintf('log barrier k = %d\n',k); 
                x_opt = inf_newton_solver(H,g,P,h,C,b,k,z,nw);
                k = mu*k;
                z = x_opt;
            end
            
        end
        function [x_opt] = mpc_fixed_log(obj,k)
            nw = [];
            [z] = initialize(obj);
            [H,g] = objective_function(obj);
            [P,h] = inequality_const(obj);
            [C,b] = equality_const(obj);
            x_opt = inf_newton_solver(H,g,P,h,C,b,k,z,nw);
        end
        function [x_opt] = mpc_fixed_log_newton(obj,nw,k)
            [z] = initialize(obj);
            [H,g] = objective_function(obj);
            [P,h] = inequality_const(obj);
            [C,b] = equality_const(obj);
            x_opt = inf_newton_solver(H,g,P,h,C,b,k,z,nw);
        end
        function [x_opt] = mpc_fixed_newton(obj,nw)
            k = 1;
            mu = 1/10;
            [H,g] = objective_function(obj);
            [P,h] = inequality_const(obj);
            [C,b] = equality_const(obj);
            [z] = initialize(obj);
            while k*length(z) >= 10e-3
%                 fprintf('log barrier k = %d\n',k);
                x_opt = inf_newton_solver(H,g,P,h,C,b,k,z,nw);
                k = mu*k;
                z = x_opt;
            end
        end
    end
end