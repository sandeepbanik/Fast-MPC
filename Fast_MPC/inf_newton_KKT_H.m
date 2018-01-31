function [H,d] = inf_newton_KKT_H(H,P,h,z,k)

d_inv = (h - P*z);
n = length(d_inv);
D = zeros(n,n);

for i=1:n
   D(i,i) = (1/(d_inv(i)))^2; 
end
% D = diag((d_inv).^-2);

d = 1./d_inv;
H = 2*H + k*P'*D*P;

end