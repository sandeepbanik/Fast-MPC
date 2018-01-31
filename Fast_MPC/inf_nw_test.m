function [H,del_f] = inf_nw_test(z)
n = length(z);
H = zeros(n,n);
for i=1:length(z)
   H(i,i) =  (1/z(i));
end

del_f = zeros(n,1);
for i=1:n
   del_f(i) = log(z(i)) + 1; 
end

end