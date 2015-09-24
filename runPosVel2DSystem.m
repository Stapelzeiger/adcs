close all; clear all; clc
% Use 2D state system (rotation angle and rate)

% Simulation Parameters
t_0 = 0.0;
t_end = 10.0;
dt = 0.1; % Time step
num_steps = ceil((t_end - t_0) / dt);

% Standard deviation of measurement noise
noiseStdDev = [0.01;
			   0.01];

% Initialise the system
theta_0 = 0.0;
omega_0 = 0.1;
sys_inertia = 1.0;
System = RotationBody2D();
System.setPos(theta_0);
System.setVel(omega_0);
System.setInertia(sys_inertia);

% KF Parameters
state_names = ['Position'; 'Velocity'];

F = [1, dt
	 0, 1];
B = [0;
     0];
H = eye(2);

Q = diag([0.001, 0.001]);

% Initialisation of KF
x_0 = [0;
       0];
P_0 = 0.1 * eye(2);
KF = KalmanFilter(F, B, H);
KF.reset(x_0, P_0)

% Logging outputs
time = [t_0];
x_pred = [x_0]; % KF predictions
x_est = [x_0]; % KF corrected estimate
x_exact = [theta_0;
	       omega_0]; % Exact state of system

P_pred = zeros(2,2,num_steps+1);
P_estim = zeros(2,2,num_steps+1);
P_pred(:,:,1) = P_0;
P_estim(:,:,1) = P_0;

% Simulate rotating system and run KF
for i = 1:num_steps
	time = [time, i * dt];
	System.update(0, dt);

	x_exact = [x_exact, System.measurePosVel([0; 0])];

	KF.predict(0, Q);
	x_pred = [x_pred, KF.x];
	P_pred(:,:,i+1) = KF.P;

	KF.measure(System.measurePosVel(noiseStdDev), diag(noiseStdDev.^2));
	x_est = [x_est, KF.x];
	P_estim(:,:,i+1) = KF.P;
end

plot_KF_results(time, x_exact, x_est, P_estim, state_names)
