ekf_euler_angles = timetable(ekf_estimate.Time);
euler = q2euler([ekf_estimate.est{:,:}]);

ekf_euler_angles.yaw = euler(1,:)';
ekf_euler_angles.pitch = euler(2,:)';
ekf_euler_angles.roll = euler(3,:)';
% 
phone_estimate = timetable(shs_sample.device_computed.attitude.Time);
euler = q2euler([shs_sample.device_computed.attitude{:,:}']);

phone_estimate.yaw = euler(1,:)';
phone_estimate.pitch = euler(2,:)';
phone_estimate.roll = euler(3,:)';

figure
subplot(2,1,1)
stackedplot(ekf_euler_angles)
title('EKF estimate')
subplot(2,1,2)
stackedplot(phone_estimate)
title('Phone Estimate')
set(gcf,'position',[ 1434    138    548   489])


%%
clear cov_euler
cov_quat = estimate1.P;
q = estimate1.est;
for i = 1:height(estimate1)
    cov_euler{i,1} = Gq(q{i})*cov_quat{i}*Gq(q{i})';
end
%
for i = 1:height(estimate1)
    diag_cov_euler(:,i) = diag(cov_euler{i});
    
end
figure();
stackedplot(diag_cov_euler');

%%
tester = estimate1(estimate1.corrected == 1,:).type;
unique(tester)
%%
covariance = sqrt(covariance);

[yaw, pitch, roll] =  ...
    quat2angle(target.est(:,:));

[cov_plus_yaw, cov_plus_pitch, cov_plus_roll] =  ...
    quat2angle(target.est(:,:) + covariance);

[cov_min_yaw, cov_min_pitch, cov_min_roll] =  ...
    quat2angle(target.est(:,:)-covariance);


figure
subplot(3,1,1)
hold on
plot(yaw)
plot(cov_plus_yaw)
plot(cov_min_yaw)
hold off
subplot(3,1,2)
hold on
plot(pitch)
plot(cov_plus_pitch)
plot(cov_min_pitch)
hold off
subplot(3,1,3)
hold on
plot(roll)
plot(cov_plus_roll)
plot(cov_min_roll)
hold off

%%
figure()
% plot(cov_plus_yaw)
subplot(3,1,1)
plot(yaw)
subplot(3,1,2)
plot(yaw-cov_plus_yaw)
subplot(3,1,3)
plot(yaw-cov_min_yaw)

%%
figure()
stackedplot([yaw,yaw-cov_plus_yaw,yaw-cov_min_yaw])

function euler = q2euler(q)
% Q2EULER  Convert quaternions to Euler angles
% euler = q2euler(q)
% q is a quaternion in columns (4xN)
% euler = [yaw(z) ; pitch(x) ; roll(y)]
  q = [q(1,:);q(3,:);q(2,:);q(4,:)];
  euler = zeros(3, size(q, 2));

  xzpwy = q(2, :).*q(4, :) + q(1, :).*q(3, :);

	IN = xzpwy+sqrt(eps)>0.5;  % Handle the north pole
  euler(1, IN) = 2*atan2(q(2, IN), q(1, IN));
  IS = xzpwy-sqrt(eps)<-0.5;  % Handle the south pole
  euler(1, IS) = -2*atan2(q(2, IS), q(1, IS));

  I = ~(IN | IS);  % Handle the default case

  euler(1, I) = atan2(2*(q(2, I).*q(3, I) - q(1, I).*q(4, I)),...
                      1-2*(q(3, I).^2 + q(4, I).^2));


  euler(3, I) = atan2(2*(q(3, I).*q(4, I) - q(1, I).*q(2, I)),...
                      1-2*(q(2, I).^2 + q(3, I).^2));

  euler(2, :) = -asin(2*xzpwy);

  euler = mod(euler+pi, 2*pi) - pi;
end