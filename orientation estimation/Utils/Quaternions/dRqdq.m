function dRdq = dRqdq(q)
% Derivative of a rotation matrix wrt a quaternion
q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);

dRdq(:,:,1) = 2* [2*q0   -q3    q2;
                    q3  2*q0   -q1;
                   -q2    q1  2*q0];

dRdq(:,:,2) = 2* [2*q1    q2    q3;
                    q2     0   -q0;
                    q3    q0     0];
                
dRdq(:,:,3) = 2* [   0    q1    q0;
                    q1  2*q2    q3;
                   -q0    q3     0];
               
dRdq(:,:,4) = 2* [   0   -q0    q1;
                    q0     0    q2;
                    q1    q2  2*q3];
end