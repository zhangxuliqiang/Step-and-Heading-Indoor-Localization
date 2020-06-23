function [acc_invD,acc_bias] = accCalib(data)

N = size(data,1);
M=ones(N,13);
% data = data./9.81;

for i=1:N
    M(i,1:9)=kron(data(i,:),data(i,:));
    M(i,10:12)=data(i,:);
end

cvx_begin
    variable A(3,3)
    variable b(3,1)
    variable c(1,1)
    minimize( norm( M * [vec(A) ; b ; c] , 2 ) )
    subject to
    trace(A) == 1
    A-0.0001*eye(3) == semidefinite(3)
cvx_end

invDT_invD = inv(0.25 * ( b' / A * b ) - c) * A;
acc_invD = chol(invDT_invD);

acc_bias = -0.5* ( A \ b );
% acc_bias = acc_bias.*9.81;
end