
ekf_estimate_tt = table2timetable(ekf_estimate); 

ekf_estimate_retime = ekf_estimate_tt(shs_sample.device_computed.attitude.Time, :);

[~,ia,~]   = unique(ekf_estimate_retime.Time);

ekf_estimate_retime = ekf_estimate_retime(ia,:);

all_quat_angle_diff = [];
all_estimate_diff = [];
all_gt_diff = [];

initial_estimate = ekf_estimate_retime.est{1,:};
initial_estimate_conjugate =  [initial_estimate(1); -initial_estimate(2:4)];

initial_gt = shs_sample.device_computed.attitude{1,:}';
initial_gt_conjugate = [initial_gt(1); -initial_gt(2:4)];

for i = 1 :height(ekf_estimate_retime)

estimate = ekf_estimate_retime.est{i,:};

estimate_diff = quat_mult(estimate, initial_estimate_conjugate);

gt = [shs_sample.device_computed.attitude{i,:}]';

gt_diff = quat_mult(gt, initial_gt_conjugate);

gt__diff_conjugate = [gt_diff(1); -gt_diff(2:4)];

quat_angle_diff = quat_mult(estimate_diff,gt__diff_conjugate);

all_estimate_diff = [all_estimate_diff, estimate_diff];
all_gt_diff = [all_gt_diff, gt_diff];

all_quat_angle_diff = [all_quat_angle_diff, quat_angle_diff ];

end

%%
euler = quat2eul(all_quat_angle_diff');

euler_gt_diff = quat2eul(all_gt_diff');
euler_estimate_diff = quat2eul(all_estimate_diff');

figure
subplot(2,1,1) 
stackedplot(euler_gt_diff)
subplot(2,1,2)
stackedplot(euler_estimate_diff)


%%
figure
hold on
plot(euler_gt_diff(:,1))
plot(euler_estimate_diff(:,1))
hold off
%%
figure
hold on
plot(euler_gt_diff(:,1) - euler_estimate_diff(:,1))
plot(-euler(:,1))
hold off
%%
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

%%
figure
subplot(2,1,1)
stackedplot(ekf_euler_angles)
title('EKF estimate')
subplot(2,1,2)
stackedplot(phone_estimate)
title('Phone Estimate')
set(gcf,'position',[ 1434    138    548   489])

%% 
title('SHS trajectory of trial 3')

%%
figure
subplot(3,1,1)
hold on
plot(ekf_euler_angles.Time, ekf_euler_angles.yaw)
plot(phone_estimate.Time, phone_estimate.yaw)
hold off
legend('ekf','phone estimate')
xlabel('Time')
ylabel('Angle (rad)')
title('Yaw')
subplot(3,1,2)
hold on
plot(ekf_euler_angles.Time, ekf_euler_angles.roll)
plot(phone_estimate.Time, phone_estimate.roll)
hold off
legend('ekf','phone estimate')
xlabel('Time')
ylabel('Angle (rad)')
title('Roll')
subplot(3,1,3)
hold on
plot(ekf_euler_angles.Time, ekf_euler_angles.pitch)
plot(phone_estimate.Time, phone_estimate.pitch)
hold off
legend('ekf','phone estimate')
xlabel('Time')
ylabel('Angle (rad)')
title('Pitch')
sgtitle([ ' Orientation Estimation vs Smartphone Estimate'])

%%

phone_start_angle = ones(height(phone_estimate),1)*phone_estimate(1,:).yaw;
phone_estimate.angle_diff = angdiff(phone_start_angle, [phone_estimate.yaw]);

ekf_start_angle = ones(height(ekf_euler_angles),1)*ekf_euler_angles(1,:).yaw;
ekf_euler_angles.angle_diff = angdiff(ekf_start_angle, [ekf_euler_angles.yaw]);

%%

phone_estimate_retime = retime(phone_estimate,unique(ekf_euler_angles.Time),'linear');

[~,ia,~] = unique(ekf_euler_angles.Time);

ekf_euler_angles_retime = ekf_euler_angles(ia,:);

%%
figure
hold on
plot(phone_estimate_retime.Time, phone_estimate_retime.angle_diff)
plot(ekf_euler_angles_retime.Time, ekf_euler_angles_retime.angle_diff)
hold off

angle_diff_ekf_phone = angdiff(phone_estimate_retime.angle_diff, ekf_euler_angles_retime.angle_diff);
%%
figure
hold on
plot(phone_estimate_retime.Time,  angle_diff_ekf_phone)
plot(ekf_estimate_retime.Time, euler_gt_diff(:,1) - euler_estimate_diff(:,1))
plot(ekf_estimate_retime.Time, -euler(:,1))
%%
close all
figure
stackedplot(shs.sl_components,'omitnan')

figure
stackedplot(shs.step_and_orient.step_length)

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

function output = quat_mult(p,q)
p0 = p(1);pv = p(2:4); q0 = q(1); qv = q(2:4);

temp1 = p0*q0 - dot(pv,qv);
temp2 = p0*qv + q0*pv + cross(pv,qv);

output = [temp1;
          temp2];
end