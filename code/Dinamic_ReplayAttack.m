%% Vehicle Dynamics and Replay Attack Simulation

%% Replay Attack Setup
tAttack = tBrake;                 % Attack starts at t = 30 s
kAttack = round(tAttack/Ts) + 1;

% Buffer recording during steady-state window t in [15s, 30s)
tRecStart = 15;
kRecStart = round(tRecStart/Ts) + 1;
kRecEnd   = kAttack - 1;
lenBuffer = kRecEnd - kRecStart + 1;

v2_buffer = zeros(1, lenBuffer);
a2_buffer = zeros(1, lenBuffer);

% Wireless DSRC Network Communication Channels
v_net2 = zeros(1, K);
a_net2 = zeros(1, K);
net_Delta_a0 = Delta_a0;          % Explicit broadcast authentication channel

%% Main loop
for k = 1:K-1
    % --- LEADER DYNAMICS (Vehicle 1) ---
    if t(k) < tBrake
        a_nom_leader = 0;
    elseif v(1,k) > 0
        a_nom_leader = aBrake;
    else
        a_nom_leader = 0;
    end
    
    % Leader control signal with noisy authentication signal
    a(1,k) = a_nom_leader + Delta_a0(k);
    
    % Discrete state update for Leader
    v(1,k+1) = v(1,k) + Ts * a(1,k);
    x(1,k+1) = x(1,k) + Ts * v(1,k);

    %% REPLAY ATTACK MANAGEMENT ON VEHICLE 2
    % Buffer recording in steady state [15s, 30s]
    if k >= kRecStart && k <= kRecEnd
        v2_buffer(k - kRecStart + 1) = v(2,k);
        a2_buffer(k - kRecStart + 1) = a(2,k);
    end

    % DSRC Network Transmission from Vehicle 2 (Eq. 13 vs Normal)
    if k < kAttack
        v_net2(k) = v(2,k);
        a_net2(k) = a(2,k);
    else
        % Cyclic replaying of recorded steady-state signals
        idxLoop = mod(k - kAttack, lenBuffer) + 1;
        v_net2(k) = v2_buffer(idxLoop);
        a_net2(k) = a2_buffer(idxLoop);
    end

    %% FOLLOWERS DYNAMICS (Vehicles 2...N)
    for i = 2:N
        d(i,k) = x(i-1,k) - x(i,k);
        e(i,k) = d(i,k) - h * v(i,k);
        
        if i == 3
            % Vehicle 3 receives tampered/replayed data from Vehicle 2
            v_prec = v_net2(k);
            a_prec = a_net2(k);
        else
            % Genuine DSRC data transmission
            v_prec = v(i-1,k);
            a_prec = a(i-1,k);
        end

        % Discrete CACC control law
        a(i,k+1) = (Ts*kp/h) * d(i,k) ...
                 - Ts*(kp + kd/h) * v(i,k) ...
                 + (1 - Ts*(kd + 1/h)) * a(i,k) ...
                 + (Ts*kd/h) * v_prec ...
                 + (Ts/h) * a_prec;

        % Kinetic update
        v(i,k+1) = v(i,k) + Ts * a(i,k);
        x(i,k+1) = x(i,k) + Ts * v(i,k);
    end
end

% Compute final step K
a_net2(K) = a_net2(K-1);
for i = 2:N
    d(i,K) = x(i-1,K) - x(i,K);
    e(i,K) = d(i,K) - h * v(i,K);
end

%% Collision Check
collision_idx = find(d(3, :) <= 0, 1);
if ~isempty(collision_idx)
    fprintf('COLLISION DETECTED: Vehicle 3 hit Vehicle 2 at t = %.2f s (v3 = %.2f m/s).\n', ...
            t(collision_idx), v(3, collision_idx));
else
    fprintf('No collision detected in the simulation duration.\n');
end
