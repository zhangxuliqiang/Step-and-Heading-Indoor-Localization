% Illustrating the use of the marginalized (a.k.a. the Rao-Blackwellized)
% particle filter by solving a specific example.
%
% All the references to equations within this code are with respect to the 
% paper,
% 
% Thomas Schön, Fredrik Gustafsson, and Per-Johan Nordlund. Marginalized 
% Particle Filters for Mixed Linear/Nonlinear State-Space Models. IEEE 
% Transactions on Signal Processing, 53(7):2279-2289, Jul. 2005.
%
% If you use this code for academic work, please reference the above paper. 
% 
% Note that this code has been written in order to facilitate the 
% understanding of the Rao-Blackwellized particle filter, via solving a 
% specific example. Hence, the code is not written not to obtain a general
% implementation, nor is it written to obtain the fastest and most optimized 
% code possible.
%
% Written by:
%              Thomas Schön (schon@isy.liu.se)
%              Division of Automatic Control
%              Linköping University
%              www.control.isy.liu.se/~schon
%              Last revised on August 19, 2011
%

clear;
%============================
%===   Define the model   ===
%============================
m.f  = inline('[1*atan(x(1,:)) + 1*x(2,:); 1*x(2,:) + 0.3*x(3,:); 0.92*x(3,:)-0.3*x(4,:); 0.3*x(3,:)+0.92*x(4,:)]');  % Dynamic model
m.h  = inline('[(0.1*x(1,:).^2).*sign(x(1,:)); x(2,:) - x(3,:) + x(4,:)]');           % Measurement model
m.x0 = zeros(4,1);             % Initial state
m.P0 = (1e-6)*eye(4);          % Covariance for the initial state
m.P0(1,1) = 1;
m.R  = 0.1*eye(2);             % Measurement noise covariance
m.Q  = 0.01*eye(4);            % Process noise covariance

m.nx = 4;                      % State dimension
m.nxn = 1;                     % Nonlinear state dimension
m.nxl = 3;                     % Linear state dimension
m.ny = 2;                      % Measurement dimension

% Define model to be used in the MPF, see eq. (18-19).
m.An   = [1 0 0];
m.Al   = [1  0.3    0;
          0  0.92 -0.3; 
          0  0.3  0.92];
m.C    = [0 0  0;
          1 -1 1];

%==============================
%===   Simulate the model   ===
%==============================
Tfinal = 200;                  % Number of samples
MC     = 10;                   % Number of Monte Carlo simulations
for i=1:MC
  x = zeros(m.nx,Tfinal+1);
  y = zeros(m.ny,Tfinal);
  x(:,1) = m.x0 + sqrtm(m.P0)*randn(m.nx,1);
  for t=1:Tfinal
    x(:,t+1) = feval(m.f,x(:,t))+sqrtm(m.Q)*randn(m.nx,1);
    y(:,t)   = feval(m.h,x(:,t))+sqrtm(m.R)*randn(m.ny,1);
  end
  z{i}.xTrue = x(:,1:Tfinal);     % Store the true states
  z{i}.y     = y(:,1:Tfinal);     % Store the measurements
end;

%==========================================================
%===   Compute the estimates using the PF and the MPF   ===
%==========================================================
Npf  = 200;           % Number of particles in the particle filter
Nmpf = Npf;           % Number of particles in the MPF
for i=1:MC
  disp(['Monte Carlo iteration: ', num2str(i)])
  gPF{i}  = pf(z{i}.y,m,Npf);   % Run the standard particle filter
  gMPF{i} = mpf(z{i}.y,m,Nmpf); % Run the standard particle filter
end;

%============================
%===   Show the results   ===
%============================
figure(1)
subplot(2,1,1)
plot(gPF{1}.xf(1,:),'b')
hold on;
title('Particle filter')
plot(z{1}.xTrue(1,:),'r')
xlabel('x1')
hold off;
legend ('PF','True')
subplot(2,1,2)
plot(gPF{1}.xf(2,:),'b')
hold on;
plot(z{1}.xTrue(2,:),'r')
xlabel('x2')
hold off;

figure(2)
subplot(2,1,1)
plot(gMPF{1}.xf(1,:),'b')
hold on;
title('Rao-Blackwellized particle filter');
plot(z{1}.xTrue(1,:),'r')
xlabel('x1')
legend ('RBPF','True')
hold off;
subplot(2,1,2)
plot(gMPF{1}.xf(2,:),'b')
hold on;
plot(z{1}.xTrue(2,:),'r')
xlabel('x2')
hold off;

% Compute RMSE values
RMSE_PF  = zeros(4,1);
RMSE_MPF = zeros(4,1);
for i=1:MC
  estErrorPF  = gPF{i}.xf - z{i}.xTrue;
  estErrorMPF = gMPF{i}.xf - z{i}.xTrue;
  RMSE_PF     = RMSE_PF  + sum(estErrorPF.^2,2);
  RMSE_MPF    = RMSE_MPF + sum(estErrorMPF.^2,2);
end;
RMSE_PF  = sqrt((1/(Tfinal*MC))*RMSE_PF);
RMSE_MPF = sqrt((1/(Tfinal*MC))*RMSE_MPF);

disp(['RMSE for standard particle filter: '])
RMSE_PF
disp(['RMSE for the Rao-Blackwellized (marginalized) particle filter: '])
RMSE_MPF
