function [f0, J] = finite_difference_jacob( fun, x0 )
% make sure x0 is a column vector
[Nx,cols] = size(x0);
if cols ~= 1
    error('x0 needs to be a column vector');
end

% make sure fun returns a column vector
f0 = fun(x0);
[Nf,cols] = size(f0);
if cols ~= 1
    error('fun needs to return a column vector');
end

% initialize empty J
J = zeros(Nf, Nx);

% perform the finite difference jacobian evaluation
h = 1e-6;
for k = 1:Nx
    x = x0;
    x(k) = x(k) + h;
    f = fun(x);
    grad = (f - f0)/h;
    J(:,k) = grad;
end

% % initialize empty J
% J = zeros(Nf, Nx);
% h = 1e-15;
% for k = 1:Nx
%     x = x0;
%     x(k) = x(k) + 1i*h;
%     f = fun(x);
%     grad = imag(f)/h;
%     J(:,k) = grad;
% end
