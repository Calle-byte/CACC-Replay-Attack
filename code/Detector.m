%% Replay Attack Detection with Virtual Model & Cross-Correlator

%% A. Construction of System Matrices A, B, C for Detector Sigma_3
% State vector x_bar = [v0; d1; v1; a1; d2; v2; a2] (dimension 7 for i=3)
c_a = Ts * kp / h;
c_b = -Ts * (kp + kd / h);
c_c = 1 - Ts * (kd + 1 / h);
c_d = Ts * kd / h;
c_e = Ts / h;

% Matrix A from Equation (12)
A_virt = [ ...
    1,     0,     0,   0,     0,     0,   0; ...   % v0
    Ts,    1,   -Ts,   0,     0,     0,   0; ...   % d1
    0,     0,     1,  Ts,     0,     0,   0; ...   % v1
    c_d, c_a,   c_b, c_c,     0,     0,   0; ...   % a1
    0,     0,    Ts,   0,     1,   -Ts,   0; ...   % d2
    0,     0,     0,   0,     0,     1,  Ts; ...   % v2
    0,     0,   c_d, c_e,   c_a,   c_b, c_c];      % a2

% Input vector B and Output vector C
B_virt = [Ts; 0; 0; c_e; 0; 0; 0];                 % Driven by Delta_a0
C_virt = [0, 0, 0, 0, 0, 0, 1];                    % Extracts estimated a2

%% B. Run Virtual Model Simulation
% Initial state vector matching steady-state conditions
x_v = zeros(7, K);

x_v(1,1) = v0_leader;
x_v(2,1) = h * v0_leader;
x_v(3,1) = v0_leader;
x_v(4,1) = 0;
x_v(5,1) = h * v0_leader;
x_v(6,1) = v0_leader;
x_v(7,1) = 0;
a_hat_2 = zeros(1, K);

for k = 1:K-1
    % Evolution of virtual state
    x_v(:, k+1) = A_virt * x_v(:, k) + B_virt * net_Delta_a0(k);
    a_hat_2(k) = C_virt * x_v(:, k);
end
a_hat_2(K) = C_virt * x_v(:, K);


%% C. Cross-Correlator Computation
g_k = zeros(length(win_sec_list), K);
product_signal = a_net2 .* a_hat_2;

for w_idx = 1:length(win_sec_list)
    W = round(win_sec_list(w_idx) / Ts);
    for k = 1:K
        first_idx = max(1, k-W+1);
        win_range = first_idx:k;
        
        % Somma accumulata nella finestra mobile
        g_k(w_idx, k) = max(0,sum(product_signal(win_range)));
    end
end


%% Plot Detection Results
idx_range = (t >= 20 & t <= 40);
max_val_12s = max(g_k(3, idx_range));

for w_idx = 1:length(win_sec_list)   
    g_k(w_idx, :) = smoothdata(g_k(w_idx, :), 'gaussian', round(0.8/Ts));
end

figure('Name', 'Replay Attack Detection - Cross Correlator', 'Color', 'w');
plot(t, g_k(1, :), 'LineWidth', 1.8, 'Color', [0, 0.4470, 0.7410]); hold on;
plot(t, g_k(2, :), 'LineWidth', 1.8, 'Color', [0.8500, 0.3250, 0.0980]);
plot(t, g_k(3, :), 'LineWidth', 1.8, 'Color', [0.9290, 0.6940, 0.1250]);

xline(tAttack, '--r', 'LineWidth', 1.2, 'HandleVisibility', 'off');
grid on;
xlim([0 50]);

ylim([-0.0005, max_val_12s * 1.1]);

xlabel('time [s]', 'FontSize', 11);
ylabel('cross correlation g_k', 'FontSize', 11);
title('cross correlator', 'FontSize', 12, 'FontWeight', 'bold');
legend({'window 3 sec', 'window 8 sec', 'window 12 sec'}, ...
       'Location', 'northeast', 'FontSize', 10);