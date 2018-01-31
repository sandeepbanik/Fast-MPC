function [H,g] = fast_mpc_objective(obj)
%% Parameters
T = obj.T;                       % Horizon time
n = size(obj.Q,1);           % State vector dimension
m = size(obj.R,1);         % Control vector dimension
z = zeros(T*(n+m),1);

R = obj.R;
Q = obj.Q;
Qf = obj.Qf;
q = obj.q;
r = obj.r;
qf = obj.qf;
x = obj.x0;

%% Checking initial conditions
if (size(obj.Q,1) ~= size(obj.Q,2)) ||...
        (size(obj.Qf,1) ~= size(obj.Qf,2))
    error('State stage cost must a square matrix');
elseif size(obj.R,1) ~= size(obj.R,2)
    error('Control stage cost must a square matrix');
end


% Checking state and control linear stage cost size
if isempty(obj.q) == 0
    if (size(obj.q,1) ~= size(obj.Q,1))
        error('Linear state cost needs to be a vector of size n');
    end
else
    q = zeros(size(Q,1),1);
end
    
if isempty(obj.r) == 0
    if (size(obj.r,1) ~= size(obj.R,1))
        error('Linear control cost needs to be a vector of size n');
    end
else
    r = zeros(size(R,1),1);
end
if isempty(obj.qf) == 0
    if (size(obj.qf,1) ~= size(obj.Q,1))
        error('State terminal linear cost needs to be a vector of size n');
    end
else
    qf = zeros(size(Q,1),1);
end

%% Objective construction
H = zeros(T*(n+m),T*(n+m));
for i=m+1:(n+m):size(H,1)-n
    H(i:i+(n+m)-1,i:i+(n+m)-1) = [Q zeros(n,m); zeros(m,n) R];
end
H(1:m,1:m) = R;
H(end-n+1:end,end-n+1:end) = Qf;

g = zeros(T*(n+m),1);
for i=m+1:(n+m):size(g,1)
    if i == T*(n+m) -n+1
        g(end-n+1:end) = qf;
    else
        g(i:i+(n+m)-1) = [q;r];
    end
end
g(1:m) = r;

end