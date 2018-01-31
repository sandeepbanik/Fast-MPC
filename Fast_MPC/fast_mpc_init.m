function [z_init] = fast_mpc_init(obj)
%% Parameters
T = obj.T;
n = size(obj.Q,1);
m = size(obj.R,1);
x_min = obj.x_min;
x_max = obj.x_max;
u_min = obj.u_min;
u_max = obj.u_max;

%% Initialization step
if isempty(obj.x_init) ~= 1
    if size(obj.x_init,1) ~= T*(n+m)
        error('Initialization size mismatch (T*(n+m))');
    else
        z_init = obj.x_init;
    end
else
    x_init = (x_min + x_max)/2;
    u_init = (u_min + u_max)/2;
    z_init = zeros(T*(m+n),1);
    for i = 1:(m+n):T*(m+n)-(m+n)+1
       z_init(i:i+m-1,1) = u_init;
       z_init(i+m:i+m+n-1,1) = x_init;
    end
   
end
end