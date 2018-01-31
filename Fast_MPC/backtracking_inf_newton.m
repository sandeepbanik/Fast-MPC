function [z,nu] = backtracking_inf_newton(z,nu,del_z,del_nu,rp,rd,al,bt)
t = 1;
too_many_steps_counter = 1000;
while norm([rp(z + t*del_z);rd(z+t*del_z,nu+t*del_nu)],2) > (1-al*t)*norm([rp(z);rd(z,nu)],2)
    t = bt*t;
    if too_many_steps_counter == 0
        error('line search fail - took too many iterations');
    end
end
z = z + t*del_z;
nu = nu + t*del_nu;

end