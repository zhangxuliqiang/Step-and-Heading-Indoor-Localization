function estimate = MultiplicativeExtendedKalmanFilter_series(est, acc,gyr,mag,magnetic, variance, debug_flag)

Types = categorical({'GYR','ACC', 'MAG'});

gyr.Type(:) = Types(1);
acc.Type(:) = Types(2);
mag.Type(:) = Types(3);

combined_raw = [acc; mag; gyr];
combined_raw = sortrows(combined_raw);

P = 0.01 * eye(3);

calAcc_R = variance.acc * eye(3);

calGyr_R = variance.gyr * eye(3);

calMag_R = variance.mag * eye(3);

combined = [combined_raw.X, combined_raw.Y, combined_raw.Z]';
type = [combined_raw.Type];

dT = [0; seconds(diff(gyr.Time))];
Time = [seconds(combined_raw.Time)];

    
estimate = repmat(struct('Time', nan, ...
                         'est',nan, ...
                         'P', nan,...
                         'error', nan,...
                         'type', nan,...
                         'corrected', nan),...
                         height(combined_raw), 1 );


g = [0; 0; -9.81];

mag_vector = mean(magnetic{:,:});
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
            % Gyro motion model
            counter = counter +1;
            
            % -------------  MOTION UPDATE -----------------------
            if counter > 1
            est = quat_mult(est,exp_q(0.5*dT(counter)* y));
            G = dT(counter)*quat2rotmat(est);
            P = P + G*calGyr_R*G';
            end
            
            
    % -------------- MEASUREMENT UPDATES ------------------
       
            
        case Types(2)
            % Accelerometer measurement update
           y(3) = -y(3);
            H_acc = -quat2rotmat(est)*vec_x(g);

            H = [H_acc];

            y_hat = [-quat2rotmat(est)' * g];

            R = [calAcc_R];

            error = y - y_hat;
            
            if norm(y) < 10.3 && norm(y) > 9.3
                [est,P] = measUpdate(est,P,error,H,R);            
                corrected = 1;
            end
            
         case Types(3)
             % magnetometer measurement update
            
            H_mag = quat2rotmat(est)*vec_x(mag_field) ;

            H = [H_mag];

            y_hat = [quat2rotmat(est)' * mag_field];

            R = [calMag_R];

            error = y - y_hat;
            
             if norm(y) < 1.1 && norm(y) > 0.9
                 [est,P] = measUpdate(est,P,error,H,R);
                 corrected = 1;
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
estimate = table2timetable(estimate);
    
end

function [est,P] = measUpdate(est,P,error,H,R)
    % EKF measurement update
    S = H*P*H' + R;
    K = (P*H') / S;
    P = P - K*S*K'; 
    eta = K*error;
    
    %relinearize
    est = quat_mult(exp_q(eta/2),est);
    
end

function R = quat2rotmat(q)
    % Convert a quaternion to a rotation matrix
    q0 = q(1); qv = q(2:4);
    R = qv*qv'+q0*eye(3)+2*q0*vec_x(qv)+vec_x(qv)^2;
end


function output = quat_mult(p,q)
p0 = p(1);pv = p(2:4); q0 = q(1); qv = q(2:4);

temp1 = p0*q0 - dot(pv,qv);
temp2 = p0*qv + q0*pv + cross(pv,qv);

output = [temp1;
          temp2];
end

function output = exp_q(eta)

temp1 = cos(norm(eta));
temp2 = (eta/norm(eta))*sin(norm(eta));

output = [temp1;
          temp2];
end

function output = vec_x(u)

output = [0, -u(3), u(2);
          u(3), 0, -u(1);
          -u(2), u(1), 0];
end
