function [mag_D, mag_bias]= magCalib(raw)
%Estimate the local field norm from calibration position 

local_field_norm=mean(vecnorm(raw'));
raw=raw./local_field_norm;

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
    minimize( norm(M*[vec(A); b; c]) )
    subject to
    trace(A) == 1
    A-0.0001*eye(3) == semidefinite(3)
cvx_end

DTD = inv(0.25 * ( b' / A * b ) - c) * A;
D = chol(DTD);
bias = -0.5* ( A \ b );

%Compensate for normalization effects in calibration
mag_bias= bias.*local_field_norm;
mag_D =D;

end