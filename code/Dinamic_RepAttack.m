%% 2. Vehicle Dynamics and CACC Control Simulation WITH REPLAY ATTACK
% Run 'Inizializzazione_Platoon.m' first.
% Outputs the same state variables (x, v, a, d, e) so 'Plot_Risultati.m' can be reused directly.

if ~exist('K', 'var')
    error('Please run Inizializzazione_Platoon.m first!');
end

%% Replay Attack Parameters & Buffer Setup (Paper Sec. II-B & IV)
tAttack = tBrake;                 % Replay attack starts at t = 30 s (Paper Sec. IV)
kAttack = round(tAttack/Ts) + 1;

% Recording window during steady-state (e.g., t in [15s, 30s])
tRecStart = 15;
kRecStart = round(tRecStart/Ts) + 1;
kRecEnd   = kAttack - 1;
lenBuffer = kRecEnd - kRecStart + 1;

% Buffers to store the recorded data transmitted by Vehicle 2
v2_buffer = zeros(1, lenBuffer);
a2_buffer = zeros(1, lenBuffer);

% Signals actually transmitted over the DSRC network by Vehicle 2
v_net2 = zeros(1, K);
a_net2 = zeros(1, K);

%% Main Discrete Simulation Loop
for k = 1:K-1
    % --- LEADER DYNAMICS (Vehicle 1) ---
    if t(k) < tBrake
        a(1,k) = 0;
    elseif v(1,k) > 0
        a(1,k) = aBrake;
    else
        a(1,k) = 0; % Stationary once stopped
    end
    
    % Discrete state update for Leader (Equation 7 from paper)
    v(1,k+1) = v(1,k) + Ts * a(1,k);
    x(1,k+1) = x(1,k) + Ts * v(1,k);

    % --- REPLAY ATTACK MANAGEMENT ON VEHICLE 2 ---
    % 1. Record data in steady-state window [15s, 30s]
    if k >= kRecStart && k <= kRecEnd
        v2_buffer(k - kRecStart + 1) = v(2,k);
        a2_buffer(k - kRecStart + 1) = a(2,k);
    end

    % 2. Network Transmission (Normal vs Replay)
    if k < kAttack
        % Normal operation: transmit real sensor/control data
        v_net2(k) = v(2,k);
        a_net2(k) = a(2,k);
    else
        % REPLAY ATTACK: Cyclic replaying of frozen steady-state buffer
        idxLoop = mod(k - kAttack, lenBuffer) + 1;
        v_net2(k) = v2_buffer(idxLoop);
        a_net2(k) = a2_buffer(idxLoop);
    end

    % --- FOLLOWERS DYNAMICS (Vehicles 2...N) ---
    for i = 2:N
        % Actual relative physical distance and spacing error
        d(i,k) = x(i-1,k) - x(i,k);
        e(i,k) = d(i,k) - h * v(i,k);
        
        % Data received via wireless network from preceding vehicle (i-1)
        if i == 3
            % Vehicle 3 receives tampered/replayed signals from Vehicle 2
            v_prec = v_net2(k);
            a_prec = a_net2(k);
        else
            % Other vehicles receive genuine data
            v_prec = v(i-1,k);
            a_prec = a(i-1,k);
        end

        % CACC discrete control law (Equation 8 from paper)
        a(i,k+1) = (Ts*kp/h) * d(i,k) ...
                 - Ts*(kp + kd/h) * v(i,k) ...
                 + (1 - Ts*(kd + 1/h)) * a(i,k) ...
                 + (Ts*kd/h) * v_prec ...
                 + (Ts/h) * a_prec;

        % Kinetic state update (Equation 7 from paper)
        v(i,k+1) = v(i,k) + Ts * a(i,k);
        x(i,k+1) = x(i,k) + Ts * v(i,k);
    end
end

% Compute values at final time step K
for i = 2:N
    d(i,K) = x(i-1,K) - x(i,K);
    e(i,K) = d(i,K) - h * v(i,K);
end

fprintf('Vehicle dynamics simulation WITH REPLAY ATTACK completed.\n');