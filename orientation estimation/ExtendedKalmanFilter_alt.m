function estimate = ExtendedKalmanFilter_alt(prior_est, accSampled,gyrSampled,magSampled,magnetic, variance, debug_flag)



prior_P = 0.01 * eye(4);

calAcc_R = variance.acc * eye(3);

calGyr_R = variance.gyr * eye(3);

calMag_R = variance.mag * eye(3);

acc = accSampled{:,:}';
gyro = gyrSampled{:,:}'; 
mag = magSampled{:,:}';

% gyroscope time update

dT = [0; seconds(diff(accSampled.Time))];

% transpose measurements to get column vectors for matrix operations
% gyro = [data.gyr_X, data.gyr_Y, data.gyr_Z]';
% acc = [data.acc_X, data.acc_Y, data.acc_Z]';
% mag = [data.mag_X, data.mag_Y, data.mag_Z]';
Time = [seconds(accSampled.Time)];

    
estimate = repmat(struct('Time', nan, ...
                         'final_q',nan, ...
                         'final_P', nan,...
                         'error', nan),...
                         height(accSampled), 1 );


g = [0; 0; -9.81];

% dip_angle = 67.095;
% mag_field = [cosd(dip_angle);sind(dip_angle); 0 ];
% mag_field =  magnetic.vector./norm(magnetic.vector);

mag_vector = mean(magnetic{:,:});
% mag_vector(3) = 0;
mag_field =  transpose(mag_vector./norm(mag_vector));

for index = 1:1:height(accSampled)
    
    if (mod(index/height(accSampled),0.1) < 0.00001)
        disp(['percentage complete: ', num2str(index/height(accSampled))])
    end
    % -------------  MOTION UPDATE -----------------------
    F = eye(4) + 0.5*(dT(index)* Somega(gyro(:,index)));
    Gu= dT(index)./ 2 *Sq(prior_est);
    prior_est = F* prior_est;    
    prior_est = prior_est / norm(prior_est);
    prior_P = F*prior_P*F' + Gu*calGyr_R*Gu';
    
    if debug_flag
        x.prior_est = prior_est;
    end
    % -------------- MEASUREMENT UPDATES ------------------
    
    % Accelerometer measurement update
    dRdq_acc = dRqdq(prior_est);
    H_acc = [-dRdq_acc(:,:,1)'*g,...
             -dRdq_acc(:,:,2)'*g,...
             -dRdq_acc(:,:,3)'*g,...
             -dRdq_acc(:,:,4)'*g];
         
    dRdq_mag = dRqdq(prior_est);
    H_mag = [dRdq_mag(:,:,1)'*mag_field, ...
             dRdq_mag(:,:,2)'*mag_field, ...
             dRdq_mag(:,:,3)'*mag_field, ...
             dRdq_mag(:,:,4)'*mag_field];
         
    H = [H_acc; H_mag];
    
    y = [acc(:,index); mag(:,index)];
    
    y_hat = [-quat2rotmat(prior_est)' * g; quat2rotmat(prior_est)' * mag_field];
    
    R = [calAcc_R, zeros(3);
         zeros(3), calMag_R];
    
    error = y - y_hat;
    
    S = H*prior_P*H' + R;
    K = (prior_P*H') / S;
    post_P = prior_P - K*S*K';
    post_est = prior_est + K* error;
    
    final_q  = post_est/norm(post_est);
%     final_q = final_q * sign(final_q(1));
    J = (1/norm(post_est)^3)*(post_est*post_est');
    final_P = J*post_P*J';
    
    prior_est = final_q;
    prior_P = final_P;
        
%     --------------- SAVING ESTIMATE COMPONENTS ---------
    x.Time =Time(index);
    x.final_q = final_q;
    x.final_P = final_P;
    x.error = error;

    
    estimate(index) = x;
    
end

estimate = struct2table(estimate);
estimate.Time = seconds(estimate.Time);
estimate = table2timetable(estimate);
    
end