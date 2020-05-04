%% Control Parameters
clear all
close all 

sampling_freq = 1000; % Hz
log_file_name = "sensorLog_20200224T154349_10hz_atdesk.txt";
visualize = true;

calAcc = [];
calGyr = [];
calMag = [];
%% Import data
opts = delimitedTextImportOptions("NumVariables", 6);

% Specify range and delimiter
opts.DataLines = [1, Inf];
opts.Delimiter = "\t";

% Specify column names and types
opts.VariableNames = ["Time", "Type", "x", "y", "z", "q4"];
opts.VariableTypes = ["double", "categorical", "double", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, "Type", "EmptyFieldRule", "auto");

% Import the data
phone_imu_data = readtable("/home/vaningen/MEGAsync/MSc Sensor Fusion Thesis/Code/Lab Session Answers/" + log_file_name, opts);

% Clear temporary variables
clear opts
%%
figure(1);
subplot(1, 2, 1);
ownView = OrientationView('Own filter', gca);  % Used for visualization.
ownView.activateOutlierControl;
googleView = [];

%% Filter settings
t0 = [];      % Initial time (initialize on first data received)

% If not indicated then default mean and covariance are used
if isempty(calAcc)
    calAcc.m = zeros(3, 1);
    calAcc.R = 1e-2 * eye(3);
end
if isempty(calGyr)
    calGyr.m = zeros(3, 1);
    calGyr.R = 1e-5 * eye(3);
end
if isempty(calMag)
    calMag.m = zeros(3, 1);
    calMag.R = 0.5 * eye(3);
end

magStat = []; % Samples used to estimate the earth magnetic field
gyrStat = []; % Samples used to estimate the gyroscope bias
mg = nan*zeros(3,1);  % uT

% Flag to indicate that we want to estimate the earth magnetic field
estMagField = 1;

% Alternative:
% estMagField = 0;
% mg = [0; sqrt(mg(1)^2 + mg(2)^2); mg(3)]; %

% State dimension: Estimating quaternions so 4
nx = 4;

% Current filter state.
x = nan(nx, 1);
P = eye(nx, nx);

% Saved filter states.
xhat = struct('t', [0],...
    'x', zeros(nx, 0),...
    'P', zeros(nx, nx, 0));

% Filter loop
initAcc = nan(3, 1);
initMag = nan(3, 1);
i = 1;

for index = 1 : height(phone_imu_data)
    
    sample = phone_imu_data(index,:);
    
    % convert from millisec to seconds
    t = sample.Time/sampling_freq;
    
    % Compute the time step
    if isempty(t0)  % Initialize t0
        t0 = t;
        current_time = t0;
        dT = 0;
        disp("t0 initiliazed")
    else
           dT = (t - t0)  - xhat.t(end);
    end
    
    if sample.Type == "GYR"
        gyr = sample{1,3:5}';
        acc = nan;
        mag = nan;
        orientation = nan;
        
    elseif sample.Type == "MAG"
        gyr = nan;
        acc = nan;
        mag = sample{1,3:5}';
        orientation = nan;
        
    elseif sample.Type == "ACC"
        gyr = nan;
        acc = sample{1,3:5}' - calAcc.m;
        mag = nan;
        orientation = nan;
        
    elseif sample.Type == "ORI"
        gyr = nan;
        acc = nan;
        mag = nan;
        orientation = sample{1,3:end}';
    end
    
    % Estimate the initial orientation using the first acc and mag
    % samples
    % Note that the initialization relies on the assumption that at some
    % point acc and mag measurements are available
    if any(isnan(x))
        if ~any(isnan(acc))
            initAcc = acc;
        end
        if ~any(isnan(mag))
            initMag = mag;
        end
        if ~any(isnan(initAcc)) && ~any(isnan(initMag))
            x = init(initAcc, initMag);
            disp("orientation initialization has occured");
        end
        % If not both acc and mag measurements have arrived after 3
        % seconds, initialise as zero.
        if (t - t0) > 3
            x = [1;0;0;0];
        end
        continue;
    end  

    % Detect stand still (no rotation) to aid gyroscope bias estimation
    % and estimation of the earth magnetic field.
    standStill = norm(gyr) < 0.03;  

    
    %% Time update
    % If gyroscope measurements are available, do a time update using the
    % gyroscope measurement and its covariance.
    if ~any(isnan(gyr))
        
                % the difference between sample is important for orientation
        % estimation
        previous_time = current_time;
        current_time = t;
        dT = current_time - previous_time;
        
        Rw = calGyr.R;
        % Collect some data for bias estimation
        if standStill && length(gyrStat) < 100
            gyrStat = [gyrStat ; gyr'];
            calGyr.m = mean(gyrStat,1)';
        end
        % Correct the gyroscope data with the estimated bias
        gyr = gyr - calGyr.m;
        
        % perform time update
        F = eye(4) + dT/2*Somega(gyr);
        Gu= dT/2*Sq(x);
        x = F*x;
        x = x / norm(x);
        P = F*P*F' + Gu*Rw*Gu';
    end   
    
    %% Measurement Updates
       
    % Accelerometer measurement update.
    if ~any(isnan(acc))
        g = [0; 0; 9.82];
        
        % x is the rotation from body to navigation, transposing this
        % generates the rotation from navigation to body.
        
        % calculate difference between gravity mapped using estimate and 
        % acceleration 
        epsi = acc - quat2rotmat(x)' * g;
        
        dRdq = dRqdq(x);
        H = [dRdq(:,:,1)'*g, dRdq(:,:,2)'*g,...
            dRdq(:,:,3)'*g, dRdq(:,:,4)'*g];
        
        % acceleration disturbance rejection
        accDist = abs(norm(acc) - 9.81) > 1;
        
        if ~ownView.outlier_acc || ~accDist
            disp("acc measurement update")
            [x, P] = measUpdate(x,P,epsi,H,calAcc.R);
            x = x / norm(x);
        end
        ownView.setAccDist(accDist);
    end
    
    if ~any(isnan(mag))  % Magnetometer measurements are available.
        % Estimate the earthmagnetic field from the initial
        if length(magStat) < 100 && estMagField
            magStat = [magStat ; mag'];
            mg = mean(magStat, 1);
            mg = [0; sqrt(mg(1)^2 + mg(2)^2); mg(3)];
        end
        if ~any(isnan(mg))
            epsi = mag - quat2rotmat(x)' * mg;
            dRdq = dRqdq(x);
            H = [dRdq(:,:,1)'*mg, dRdq(:,:,2)'*mg,...
                dRdq(:,:,3)'*mg, dRdq(:,:,4)'*mg];
            
            % Detect mag disturbance
            Smag = H * P * H' + calMag.R;
            gateMag = epsi' / Smag * epsi; 
            magDist = gateMag > 9;
            
            if ~ownView.outlier_mag || ~magDist
                [x, P] = measUpdate(x,P,epsi,H,calMag.R);
                x  = x / norm(x);
            end
            ownView.setMagDist(magDist);
        end
    end
    

    % Visualize result
    if rem(i, 10)==0
        setOrientation(ownView, x(1:4));
        title(ownView, 'OWN', 'FontSize', 16);
        if ~any(isnan(orientation))
            if isempty(googleView)
                subplot(1, 2, 2);
                % Used for visualization.
                googleView = OrientationView('Google filter', gca);
            end
            setOrientation(googleView, orientation);
            title(googleView, 'GOOGLE', 'FontSize', 16);
        end
    end
    i = i + 1;
    
    % Save estimates
    xhat.x(:, end+1) = x;
    xhat.P(:, :, end+1) = P;
    xhat.t(end+1) = t - t0;
end

function [x,P] = measUpdate(x,P,e,H,R)
% EKF measurement update
S = H*P*H' + R;
K = (P*H') / S;
P = P - K*S*K'; % <--------- this is not how it is defined in the tutorial
x = x + K*e;
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

function x = init(acc, mag)
% Initialise orientation based on the first acc and mag samples
R = zeros(3, 3);
acc = acc / norm(acc);
mag = mag / norm(mag);

R(:, 3) = acc/norm(acc);
R(:, 1) = cross(mag, R(:, 3));  R(:, 1) = R(:, 1)/norm(R(:, 1));
R(:, 2) = cross(R(:, 3), R(:, 1)); R(:, 2) = R(:, 2)/norm(R(:, 2));
R = R';

t = R(1, 1) + R(2, 2) + R(3, 3);
r = sqrt(1+t);
s = 0.5/r;
x = [0.5*r;
    (R(3, 2) - R(2, 3))*s;
    (R(1, 3) - R(3, 1))*s;
    (R(2, 1) - R(1, 2))*s];
end
