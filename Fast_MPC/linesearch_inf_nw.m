function [x,nu] = linesearch_inf_nw(r,x,nu,del_x,del_nu)
t = 1;
al = 0.01;
beta = 0.5;
too_many_steps_counter = 1000;
while (r(x+t*del_x,nu+t*del_nu)) > (1-al*t)*r(x,nu)
    t = beta*t;
    too_many_steps_counter = too_many_steps_counter - 1;
    if too_many_steps_counter == 0
        error('line search fail - took too many iterations');
    end
end
x = x + t*del_x;
nu = nu + t*del_nu;
end