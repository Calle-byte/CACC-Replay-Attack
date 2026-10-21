%% 1. Initialization of Parameters and State Variables
% Based on the paper: Merco et al., "Replay Attack Detection in a Platoon 
% of Connected Vehicles with CACC" (ACC 2018).

%% Platoon and Simulation Parameters (Paper Sec. II & IV)
N = 6;                  % Total number of vehicles (1 Leader + 5 Followers)
Ts = 0.01;              % Sampling time [s]
Tend = 50;              % Total simulation duration [s]
t = 0:Ts:Tend;
K = numel(t);

% CACC Controller Parameters
h  = 0.6;               % Time headway [s]
kp = 1.998;             % Proportional gain CACC
kd = 1.3992;            % Derivative gain CACC

v0_leader = 20;         % Leader initial velocity [m/s]
v0_follow = 18;         % Followers initial velocity [m/s]
d0 = 10;                % Initial bumper-to-bumper distance [m]

tBrake = 30;            % Braking start time for the leader [s]
aBrake = -1;            % Leader deceleration during braking [m/s^2]

%% Allocation of State Matrices (L_i = 0 per paper Sec. II-A)
x = zeros(N, K);        % Rear bumper position [m]
v = zeros(N, K);        % Velocity [m/s]
a = zeros(N, K);        % Acceleration [m/s^2]
d = zeros(N, K);        % Inter-vehicle distance d_i = x_{i-1} - x_i [m]
e = zeros(N, K);        % Spacing error e_i = d_i - h*v_i [m]

%% Initial Conditions (Paper Sec. IV)
x(1,1) = 0;
v(1,1) = v0_leader;
for i = 2:N
    x(i,1) = x(i-1,1) - d0; % Initial positions spaced by d0
    v(i,1) = v0_follow;     % Followers initial velocity
    d(i,1) = d0;
    e(i,1) = d(i,1) - h * v(i,1); % Initial spacing error
end

fprintf('Initialization completed successfully.\n');