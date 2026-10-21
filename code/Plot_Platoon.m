%% 3. Plotting Results
% Run after 'Dinamica_Veicoli.m' or 'Dinamica_Veicoli_ReplayAttack.m'

if ~exist('v', 'var')
    error('Please run Dinamica_Veicoli.m or Dinamica_Veicoli_ReplayAttack.m first!');
end

figure('Name','CACC Vehicle Platoon Simulation Results','Color','w');
tiledlayout(3,2,'TileSpacing','compact','Padding','compact');

% Plot 1: Vehicle Velocities
nexttile;
plot(t, v, 'LineWidth', 1.4); hold on;
xline(tBrake, '--k', 'Braking Start');
grid on; xlabel('Time [s]'); ylabel('Velocity [m/s]');
title('Vehicle Velocities v_i(t)');
legend(compose('Vehicle %d', 1:N), 'Location', 'best');
ylim([-2, 23]); 

% Plot 2: Inter-vehicle Distances
nexttile;
plot(t, d(2:N,:), 'LineWidth', 1.4); hold on;
xline(tBrake, '--k', 'Braking Start');
yline(0, 'r--', 'Collision Threshold');
grid on; xlabel('Time [s]'); ylabel('Distance [m]');
title('Inter-vehicle Distances d_i(t)');
legend({'d_2 (Veh 2-1)', 'd_3 (Veh 3-2)', 'd_4 (Veh 4-3)', ...
        'd_5 (Veh 5-4)', 'd_6 (Veh 6-5)'}, 'Location', 'best');
ylim([-2, 15]); 

% Plot 3: Spacing Error e_i = d_i - h*v_i
nexttile;
plot(t, e(2:N,:), 'LineWidth', 1.4); hold on;
xline(tBrake, '--k', 'Braking Start');
yline(0, ':k');
grid on; xlabel('Time [s]'); ylabel('Error e_i [m]');
title('Spacing Error e_i(t) = d_i - h v_i');
legend(compose('Vehicle %d', 2:N), 'Location', 'best');
ylim([-1, 0.8]); 

% Plot 4: Positive Distance relative to Vehicle 1 with Collision Region & Dashed Line
nexttile;
% Positive distance behind Vehicle 1: x_1(t) - x_i(t)
x_rel1_pos = x(1, :) - x(2:N, :); 

% Check for collision (d_3 <= 0)
idx_collision = find(d(3, :) <= 0, 1, 'first'); 

if ~isempty(idx_collision)
    t_collision = t(idx_collision);
    
    % Shaded red region starting at collision time (t_collision to Tend)
    h_region = fill([t_collision Tend Tend t_collision], [-10 -10 max(x_rel1_pos(:))+10 max(x_rel1_pos(:))+10], ...
                    [1 0.8 0.8], 'EdgeColor', 'none', 'FaceAlpha', 0.5, 'DisplayName', 'Collision region');
    hold on;
    
    % Red dashed vertical line at collision onset
    h_line = xline(t_collision, '--r', 'LineWidth', 1.6, 'DisplayName', 'Collision');
end

% Plot vehicle relative distance curves
p_rel = plot(t, x_rel1_pos, 'LineWidth', 1.4); hold on;
xline(tBrake, '--k', 'Replay Attack / Braking Start');

% Configure Legend dynamically
if ~isempty(idx_collision)
    leg_labels = [compose('Vehicle %d', 2:N), {'Collision region'}];
    legend([p_rel; h_region], leg_labels, 'Location', 'best');
else
    legend(compose('Vehicle %d', 2:N), 'Location', 'best');
end

grid on; xlabel('Time [s]'); ylabel('Distance from Veh 1 [m]');
title('Positive Distance Relative to Vehicle 1 (x_1 - x_i)');
ylim([-2, max(x_rel1_pos(:)) + 5]);

% Plot 5: Vehicle Accelerations
nexttile([1 2]);
plot(t, a, 'LineWidth', 1.2); hold on;
xline(tBrake, '--k', 'Braking Start');
grid on; xlabel('Time [s]'); ylabel('Acceleration [m/s^2]');
title('Longitudinal Accelerations a_i(t)');
legend(compose('Vehicle %d', 1:N), 'Location', 'best');
ylim([-1.5, 1.8]);