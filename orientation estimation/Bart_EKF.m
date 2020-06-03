function [attitude,x] = Bart_EKF(timestamp, acc, gyr, mag, magnetic)

    disp('Estimating Orientation')

    if size(acc, 1) ~= size(gyr, 1) || size(acc, 1) ~= size(mag, 1) || size(acc, 1) ~= size(timestamp, 1) || ...
            size(acc, 2) ~= 3 || size(mag, 2) ~= 3 || size(gyr, 2) ~= 3 || size(timestamp, 2) ~= 1
        error('Bad input size of measurements vectors')
    end

    dt = [0;diff(timestamp)];
    dataLength = length(timestamp);


    %% og function
    prior_est = [1;0;0;0];
    prior_P = eye(4, 4);

    cal_acc_R = 1e-2 * eye(3);

    cal_gyr_R = 1e-5 * eye(3);

    cal_mag_R = 0.5 * eye(3);

    g = [0;0; 9.81];
    mag_field =  magnetic.vector./norm(magnetic.vector);

    for i = 1:dataLength

        % -------------  MOTION UPDATE -----------------------
        F = eye(4) + dt(i)/ 2.* Somega(gyr(i,:)');
        prior_est = F* prior_est;
        Gu= dt(i)./ 2 *Sq(prior_est);
        prior_est = prior_est / norm(prior_est);
        prior_P = F*prior_P*F' + Gu*cal_gyr_R*Gu';

        % -------------- MEASUREMENT UPDATES ------------------

        % Accelerometer measurement update
        dRdq_acc = dRqdq(prior_est);
        H_acc = [dRdq_acc(:,:,1)'*g, dRdq_acc(:,:,2)'*g,...
            dRdq_acc(:,:,3)'*g, dRdq_acc(:,:,4)'*g];

        % transpose rotation matrix of body frame to navigation frame to get from
        % navigation to body frame

        acc_error = acc(i,:)' - quat2rotmat(prior_est)' * g;

        [post_acc_P, post_acc_est] = ...
            MeasurementUpdate(acc_error, prior_est, prior_P, cal_acc_R, H_acc);

        % magnetometer measurement update
        
        % normalize magnetometer readings
        mag(i,:)= mag(i,:)/norm(mag(i,:));
        
        dRdq_mag = dRqdq(post_acc_est);
        H_mag = [dRdq_mag(:,:,1)'*mag_field, ...
            dRdq_mag(:,:,2)'*mag_field, ...
            dRdq_mag(:,:,3)'*mag_field, ...
            dRdq_mag(:,:,4)'*mag_field];

        mag_error = mag(i,:)' - quat2rotmat(post_acc_est)' * mag_field;

        [post_mag_P, post_mag_est] = ...
            MeasurementUpdate(mag_error, post_acc_est, post_acc_P, cal_mag_R , H_mag);

        x(i).euler_prior_est = q2euler(prior_est);

        prior_est = post_mag_est;
        prior_P = post_mag_P;
        %     --------------- SAVING ESTIMATE COMPONENTS ---------

        x(i).euler_post_acc_est = q2euler(post_acc_est);

        x(i).euler_post_mag_est = q2euler(post_mag_est);

        attitude(i,:) = post_mag_est';


    end

    % Come back to True North frame
        % Let's define MagRef and AccRef temporarly in Earth Magnetic Field frame
% 
%     qMagneticToTrue = dcm2quat(rotz(magnetic.declination));
%     qTrueToMagnetic = quatinv(qMagneticToTrue);
%     attitude = quatmultiply(qMagneticToTrue, attitude);
end










