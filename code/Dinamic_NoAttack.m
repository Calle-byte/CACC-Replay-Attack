%% 2. Vehicle Dynamics and CACC Control Simulation without a Replay Attack

if ~exist('K', 'var')
    error('Please run Inizialize_var.m first!');
end

for k = 1:K-1
    % --- LEADER DYNAMICS (Vehicle 1) ---
    % The leader maintains a constant velocity until t < tBrake, then decelerates until stop.
    if t(k) < tBrake
        a(1,k) = 0;
    elseif v(1,k) > 0
        a(1,k) = aBrake;
    else
        a(1,k) = 0; % Remains stationary once stopped
    end
    
    % Discrete state update for the Leader (Equation 7 from paper)
    v(1,k+1) = v(1,k) + Ts * a(1,k);
    x(1,k+1) = x(1,k) + Ts * v(1,k);

    % --- FOLLOWERS DYNAMICS (Vehicles 2...N) ---
    for i = 2:N
        % Actual relative distance and error at current step k (Equations 2 & 5)
        d(i,k) = x(i-1,k) - x(i,k);
        e(i,k) = d(i,k) - h * v(i,k);
        
        % Explicit discrete CACC control law (Equation 8 from paper)
        a(i,k+1) = (Ts*kp/h) * d(i,k) ...
                 - Ts*(kp + kd/h) * v(i,k) ...
                 + (1 - Ts*(kd + 1/h)) * a(i,k) ...
                 + (Ts*kd/h) * v(i-1,k) ...
                 + (Ts/h) * a(i-1,k);

        % Kinetic update for position and velocity (Equation 7 from paper)
        v(i,k+1) = v(i,k) + Ts * a(i,k);
        x(i,k+1) = x(i,k) + Ts * v(i,k);
    end
end

% Compute values at the final time step K
for i = 2:N
    d(i,K) = x(i-1,K) - x(i,K);
    e(i,K) = d(i,K) - h * v(i,K);
end

fprintf('Vehicle dynamics simulation completed.\n');