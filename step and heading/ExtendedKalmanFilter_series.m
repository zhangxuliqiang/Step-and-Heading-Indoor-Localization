function estimate = ExtendedKalmanFilter_series(prior_est, calib_data, ...
    variance)

acc_data = calib_data.acc_data;
gyr_data = calib_data.gyr_data;
mag_data = calib_data.mag_data;
mag_north_data = calib_data.mag_north_data ;
est = prior_est;

Types = categorical({'GYR','ACC', 'MAG'});

gyr_data.Type(:) = Types(1);
acc_data.Type(:) = Types(2);
mag_data.Type(:) = Types(3);

combined_raw = [acc_data; mag_data; gyr_data];
combined_raw = sortrows(combined_raw);

P = 1 * eye(4);

calAcc_R = variance.acc * eye(3);

calGyr_R = variance.gyr * eye(3);

calMag_R = variance.mag * eye(3);

combined = [combined_raw.X, combined_raw.Y, combined_raw.Z]';
type = [combined_raw.Type];

dT = [0; seconds(diff(gyr_data.Time))];
Time = [seconds(combined_raw.Time)];


estimate = repmat(struct('Time', nan, ...
    'est',nan, ...
    'P', nan,...
    'error', nan,...
    'type', nan,...
    'corrected', nan),...
    height(combined_raw), 1 );


g = [0; 0; -9.81];

mag_vector = mean(mag_north_data{:,:});
mag_field =  transpose(mag_vector./norm(mag_vector));

counter = 0;
progress_before= 0;
for index = 1:1:height(combined_raw)
    
    progress = round(index/height(combined_raw)*100,2);
    if mod(progress,10) == 0  && round(progress) ~= 0
        if progress~=progress_before
            disp(['percentage complete: ',  num2str(progress)]);
            progress_before = progress;
        end
    end
    
    y = combined(:,index) ;
    error = nan;
    corrected = 0;
    switch type(index)
        
        case Types(1)
            counter = counter +1;
            
            % -------------  MOTION UPDATE -----------------------
            F = eye(4) + 0.5*(dT(counter)* Somega(y));
            Gu= dT(counter)./ 2 *Sq(est);
            est = F* est;
            J = (1/norm(est)^3)*(eye(4)*(est'*est) - (est*est'));
            est = est / norm(est);
            P = F*P*F' + Gu*calGyr_R*Gu';
            %             P = J*P*J';
            % -------------- MEASUREMENT UPDATES ------------------
            
            
        case Types(2)
            % Accelerometer measurement update
            dRdq_acc = dRqdq(est);
            H_acc = [-dRdq_acc(:,:,1)'*g,...
                -dRdq_acc(:,:,2)'*g,...
                -dRdq_acc(:,:,3)'*g,...
                -dRdq_acc(:,:,4)'*g];
            
            H = [H_acc];
            
            y_hat = [-quat2rotmat(est)' * g];
            
            R = [calAcc_R];
            
            error = y - y_hat;
            
            if norm(y) < 10.0 && norm(y) > 9.6
                [est,P] = measUpdate(est,P,error,H,R);
                J = (1/norm(est)^3)*(eye(4)*(est'*est) - (est*est'));
                est = est/norm(est);
                corrected = 1;
                %                 P = J*P*J';
            end
            
        case Types(3)
            % magnetometer measurement update
            dRdq_mag = dRqdq(est);
            H_mag = [dRdq_mag(:,:,1)'*mag_field, ...
                dRdq_mag(:,:,2)'*mag_field, ...
                dRdq_mag(:,:,3)'*mag_field, ...
                dRdq_mag(:,:,4)'*mag_field];
            
            H = [ H_mag];
            
            y_hat = [ quat2rotmat(est)' * mag_field];
            
            R = [ calMag_R];
            
            error = y - y_hat;
            
            if norm(y) < 1.1 && norm(y) > 0.9
                [est,P] = measUpdate(est,P,error,H,R);
                J = (1/norm(est)^3)*(eye(4)*(est'*est) - (est*est'));
                est = est/norm(est);
                corrected = 1;
                %                  P = J*P*J';
            end
            
    end
    
    %     --------------- SAVING ESTIMATE COMPONENTS ---------
    x.Time =Time(index);
    x.est = est;
    x.P = P;
    x.error = error;
    x.type = type(index);
    x.corrected = corrected;
    
    
    estimate(index) = x;
    
end

estimate = struct2table(estimate);
estimate.Time = seconds(estimate.Time);
ekf_estimate = table2timetable(estimate);
end

function [est,P] = measUpdate(est,P,error,H,R)
% EKF measurement update
S = H*P*H' + R;
K = (P*H') / S;
P = P - K*S*K';
est = est + K*error;
end

function R = quat2rotmat(q)
% Convert a quaternion to a rotation matrix
q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
R = [2*(q0^2+q1^2) - 1  2*(q1*q2-q0*q3)    2*(q1*q3+q0*q2);
    2*(q1*q2+q0*q3)    2*(q0^2+q2^2) - 1  2*(q2*q3-q0*q1);
    2*(q1*q3-q0*q2)    2*(q2*q3+q0*q1)    2*(q0^2+q3^2) - 1];
end

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

function S=Somega(w)
% The matrix S(omega) defined in (13.11b)
wx=w(1);   wy=w(2);   wz=w(3);
S=[ 0  -wx  -wy  -wz;
    wx    0   wz  -wy;
    wy  -wz    0   wx;
    wz   wy  -wx    0];
end

function S=Sq(q)
% The matrix S(q) defined in (13.11c)
q0=q(1);   q1=q(2);   q2=q(3);   q3=q(4);
S=[-q1 -q2 -q3;
    q0 -q3  q2;
    q3  q0 -q1;
    -q2  q1  q0];
end

