function x_opt = inf_newton_solver(H,g,P,h,C,b,k,z,newton)
nu = rand(length(b),1);
iter = 1;
if isempty(newton) 
    max_iter = 1000;
else
    max_iter = newton;
end
tol = 1e-6;
for i=1:max_iter
    [KKT_H,d] = inf_newton_KKT_H(H,P,h,z,k);
    rd =@(z,v)(2*H*z + g + k*P'*d + C'*v);
    rp =@(z)(C*z - b);
    norm_grad = @(x,nu)(norm([-rd(x,nu);-rp(x)],2));
    tol_g = C*z-b;
    n_r = norm_grad(z,nu);
    n_g = norm(tol_g);
%     fprintf('iteration: %4d,   norm_residual: %.2e \n',iter, n_r);
    if ( n_r <= tol) && (n_g <=1e-8)
        x_opt = z;
        return;
    end   
    
    L = chol(KKT_H,'lower');
    optsL.LT = true;
    optsU.UT = true;
    Schur = C*linsolve(L',linsolve(L,C',optsL),optsU);
    phi_inv_rd = linsolve(L',linsolve(L,rd(z,nu),optsL),optsU);
    Beta = -rp(z) + C*phi_inv_rd;
    SL = chol(Schur,'lower');
    int_nu = linsolve(SL,-Beta,optsL);
    del_nu = linsolve(SL',int_nu,optsU);
    
    int_z = linsolve(L,-rd(z,nu)-C'*del_nu,optsL);
    del_z = linsolve(L',int_z,optsU);
    al = 10^-4;
    bt = 0.5;
    [z,nu] = backtracking_inf_newton(z,nu,del_z,del_nu,rp,rd,al,bt);
    iter = iter + 1;
    
end
x_opt = z;
end