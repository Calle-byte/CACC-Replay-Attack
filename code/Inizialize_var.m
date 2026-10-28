%% Initialization of Parameters and State Variables

%% Platoon and Simulation Parameters
N = 6;                  % Total number of vehicles (1 Leader + 5 Followers)
Ts = 0.01;              % Sampling time [s]
Tend = 50;              % Total simulation duration [s]
t = 0:Ts:Tend;
K = numel(t);

% CACC Controller Parameters
h  = 0.6;               % Time headway [s]
kp = 1.998;             % Proportional gain CACC
kd = 1.3992;            % Derivative gain CACC

% Initial Kinematic Conditions
v0_leader = 20;         % Leader initial velocity [m/s]
v0_follow = 18;         % Followers initial velocity [m/s]
d0 = 10;                % Initial bumper-to-bumper distance [m]

tBrake = 30;            % Braking start time for the leader [s]
aBrake = -1;            % Leader deceleration during braking [m/s^2]

%% Authentication Noise Signal
var_noise = 0.001;
%rng(42);
Delta_a0 = sqrt(var_noise) * randn(1, K);

% Detection Window sizes in seconds
win_sec_list = [3, 8, 12];      

%% Allocation of State Matrices
x = zeros(N, K);        % Rear bumper position [m]
v = zeros(N, K);        % Velocity [m/s]
a = zeros(N, K);        % Acceleration [m/s^2]
d = zeros(N, K);        % Inter-vehicle distance d_i = x_{i-1} - x_i [m]
e = zeros(N, K);        % Spacing error e_i = d_i - h*v_i [m]

%% Initial Conditions
x(1,1) = 0;
v(1,1) = v0_leader;
for i = 2:N
    x(i,1) = x(i-1,1) - d0;        % Initial positions spaced by d0
    v(i,1) = v0_follow;            % Followers initial velocity (18 m/s)
    d(i,1) = d0;
    e(i,1) = d(i,1) - h * v(i,1);  % Initial spacing error (-0.8 m)
end

fprintf('Initialization completed successfully\n');
