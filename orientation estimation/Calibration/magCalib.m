function [mag_invD, mag_bias]= magCalib(raw)

%Estimate calibration parameters
N=size(raw,1);
M=ones(N,13);
for i=1:N
    M(i,1:9)=kron(raw(i,:),raw(i,:));
    M(i,10:12)=raw(i,:);
end

cvx_begin
    variable A(3,3)
    variable b(3,1)
    variable c(1,1)
    minimize(norm( M * [vec(A) ; b ; c] , 2 ) )
    subject to
    trace(A) == 1
    A-0.0001*eye(3) == semidefinite(3)
cvx_end

invDT_invD = inv(0.25 * b' *inv(A) * b - c) *A;
invD = chol(invDT_invD);
bias = -0.5* inv(A) * b ;

mag_bias = bias;
mag_invD = invD;

end