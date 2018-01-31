function [C,b] = fast_mpc_eq_const(obj)

%% Parameters
A = obj.A;
B = obj.B;
w = obj.w;
n = size(A,2);
m = size(B,2);
x = obj.x0;
u = obj.R;
T = obj.T;
C = zeros(T*n,T*(n+m));
b = zeros(T*n,1);
xf = obj.x_final;

%% Initial condition check
if isempty(A)
    error('Define the state dynamics/equality constrained matrix');
elseif isempty(B)
    error('Define the control dynamics/equality constrained matrix');
end

if size(A,2) ~= size(x,1)
    error('The equality state dynamics matrix size does not match');
elseif size(B,2) ~= size(u,2)
    error('The equality control dynamics matrix size does not match');
elseif isempty(w)
    w = zeros(n,1);
end

%% Equality constraint construction

for i=n:n:T*n-n+1
    if i==n
        C(i+1:i+n,i+((i/n)-1)*(n+m+n):i+n+((i/n)-1)*(n+m+n)+m+n-1) = [-A -B eye(n)];
        b(i+1:i+n) = w;
    else
        C(i+1:i+n,((i/n)-1)*(n+m)+m+1:((i/n)-1)*(n+m)+m+m+n+n) = [-A -B eye(n)];
        b(i+1:i+n) = w;
    end
end
C(1:n,1:m+n) = [-B eye(n)];
% C(end-n+1:end,end-(n+m+n)+1:end) = [zeros(n,n) zeros(n,m) eye(n)];
b(1:n) = A*x + w;
% b(end-n+1:end) = xf;
if isempty(xf)~=1
   b = [b;xf];
   C = [C;zeros(n,size(C,2))];
end
C(end-n+1:end,end-n+1:end) = eye(n);
end