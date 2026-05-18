% AR26_steering_vs_ay.m
% Steering wheel torque and rack force as a function of lateral acceleration.
%
% Sign convention:
%   ay > 0  →  cornering right  →  right wheel is outer (loaded)
%   ay < 0  →  cornering left   →  left  wheel is outer (loaded)
%
% Can operate in two modes (set INPUT_MODE below):
%   'sweep'  — evaluates over a symmetric ay range [-AY_MAX_G, +AY_MAX_G]
%   'data'   — loads a signed lateral acceleration time history (e.g. from
%              Optimum Lap) and evaluates the moment at each sample
%
% Brush tyre model with Pacejka PDY/PKY load-dependent mu and C_Falpha.
% Inner/outer wheel split with rigid-body lateral load transfer.
% Moment path: M_kingpin -> tie rod -> rack -> pinion.

clc; clear; close all;

%% input

INPUT_MODE = 'data';    % 'sweep' or 'data'

% ── If 'data': path to Optimum Lap CSV export ─────────────────────────────────
% Optimum Lap format: metadata header rows, then [Simulation Results] section,
% then 3 column-header rows, then numeric data.
% Columns used: elapsedTime (col 2) and lateralAcceleration (col 4) in G.
% Positive ay = right turn, negative = left turn (OL convention).
DATA_FILE  = ['ar26austria.csv'];

% ── If 'sweep': symmetric range ───────────────────────────────────────────────
AY_MAX_G   = 2.5;       % g   (sweep runs from -AY_MAX_G to +AY_MAX_G)
N_AY       = 601;       % number of points  (odd -> includes zero)

% ── Diagnostic brush sweep at this ay value ───────────────────────────────────
AY_DIAG_G  = 1.5;       % g   (positive; outer = right wheel)

%%Parameters

%% vehicle params
m       = 293;          % kg   total mass
L       = 1.540;        % m    wheelbase
a_cg    = 0.853;        % m    CG to front axle
h_cg    = 0.241;        % m    CG height
t_f     = 1.410;        % m    front track (FULL width)
g_SI    = 9.81;         % m/s^2
b_cg    = L - a_cg;     % m    CG to rear axle

%% steering geometry 
e       = 15e-3;        % m    mechanical trail          estimate
L_arm   = 84.55e-3;     % m    steering arm length (LTS.m)
r_pin   = 20e-3;        % m    pinion radius

%% tire coefficients 
PDY1    =  2.1471;
PDY2    = -0.42791;
PKY1    = -31.419;
PKY2    =  1.5115;
FNOMIN  =  750;         % N
ap      =  28.55e-3;    % m    contact patch half-length

%% Pacejka functions 
mu_y_fn  = @(Fz) max(PDY1 + PDY2 .* (Fz - FNOMIN) ./ FNOMIN, 0.5);
Cfa_fn   = @(Fz) abs(PKY1) .* FNOMIN .* sin(2 .* atan(Fz ./ (PKY2 .* FNOMIN)));

%% slip angle sweep vector 
N_alpha = 600;
alphas  = linspace(0, 20, N_alpha);   % deg 


%% input daata
switch INPUT_MODE
    case 'sweep'
        % Signed sweep: negative = left turn, positive = right turn
        ay_g_vec = linspace(-AY_MAX_G, AY_MAX_G, N_AY);
        t_vec    = ay_g_vec;
        x_label  = 'Lateral acceleration a_y (g)';
        have_pos = false;

    case 'data'
        if ~isfile(DATA_FILE)
            error('Data file not found: %s\nSet DATA_FILE to your OL export.', DATA_FILE);
        end

        % ── Optimum Lap CSV parser ─────────────────────────────────────────────
        % Scans for [Simulation Results] section, skips 3 header rows, then
        % reads elapsedTime (col 2) and lateralAcceleration (col 4).
        % Falls back to generic two-column / single-column CSV if section
        % header is not found.
        fid = fopen(DATA_FILE, 'r');
        found_section = false;
        while ~feof(fid)
            raw_line = fgetl(fid);
            if contains(raw_line, '[Simulation Results]')
                found_section = true;
                break;
            end
        end

        if found_section
            % Skip 3 column-header rows (names, units-type, units)
            fgetl(fid); fgetl(fid); fgetl(fid);
            % Read remaining lines as numeric CSV
            data_str = textscan(fid, repmat('%f', 1, 27), ...
                'Delimiter', ',', 'CollectOutput', true);
            fclose(fid);
            data_mat = data_str{1};
            t_vec    = data_mat(:, 2)';   % elapsedTime [s]
            ay_g_vec = data_mat(:, 4)';   % lateralAcceleration [G]
            x_pos    = data_mat(:, 24)';  % xposition [m]
            y_pos    = data_mat(:, 25)';  % yposition [m]
            have_pos = ~all(isnan(x_pos)) && ~all(isnan(y_pos));
            fprintf('Loaded Optimum Lap data: %d samples, %.2f s lap\n', ...
                numel(t_vec), t_vec(end));
        else
            % Generic fallback: two columns [time, ay_g] or single [ay_g]
            fclose(fid);
            raw = readmatrix(DATA_FILE);
            if size(raw, 2) >= 2
                t_vec    = raw(:, 1)';
                ay_g_vec = raw(:, 2)';
            else
                ay_g_vec = raw(:, 1)';
                t_vec    = (0:numel(ay_g_vec)-1) ./ 10;
            end
            fprintf('Loaded generic CSV: %d samples\n', numel(ay_g_vec));
            have_pos = false;
        end

        % Keep signed ay throughout — sign drives left/right wheel assignment
        x_label  = 'Time (s)';
        fprintf('  ay range: [%.3f, %.3f] G\n', min(ay_g_vec), max(ay_g_vec));
end


%% Full lap 

% Physical wheel convention (fixed to car body):
%   Right wheel = positive-ay side  (outer when ay > 0, inner when ay < 0)
%   Left  wheel = negative-ay side  (outer when ay < 0, inner when ay > 0)

N_pts     = numel(ay_g_vec);
Fz_static = m * g_SI * (b_cg / L) / 2;   % N  static per front wheel (constant)

% Per-physical-wheel outputs
M_kp_R_vec  = zeros(1, N_pts);   % kingpin moment, right wheel
M_kp_L_vec  = zeros(1, N_pts);   % kingpin moment, left wheel
Fz_R_vec    = zeros(1, N_pts);
Fz_L_vec    = zeros(1, N_pts);
alpha_R_vec = zeros(1, N_pts);
alpha_L_vec = zeros(1, N_pts);
T_SW_vec    = zeros(1, N_pts);
F_rack_vec  = zeros(1, N_pts);

for k = 1:N_pts
    ay_g   = ay_g_vec(k);
    ay     = ay_g * g_SI;           % signed m/s^2
    ay_abs = abs(ay);

    % Load transfer magnitude based on |ay|
    dFz = m * ay_abs * h_cg * (b_cg / L) / (t_f);

    % Assign loads to physical wheels based on sign of ay
    if ay >= 0
        % Right turn: right wheel loaded, left wheel unloaded
        Fz_R = Fz_static + dFz;
        Fz_L = max(Fz_static - dFz, 0);
    else
        % Left turn: left wheel loaded, right wheel unloaded
        Fz_L = Fz_static + dFz;
        Fz_R = max(Fz_static - dFz, 0);
    end

    Fz_R_vec(k) = Fz_R;
    Fz_L_vec(k) = Fz_L;

    % Per-wheel Pacejka properties
    mu_R  = mu_y_fn(Fz_R);   Cfa_R = Cfa_fn(Fz_R);
    mu_L  = mu_y_fn(Fz_L);   Cfa_L = Cfa_fn(Fz_L);

    % Brush model sweep for each wheel
    Fy_R_sw = zeros(1, N_alpha);   Kp_R_sw = zeros(1, N_alpha);
    Fy_L_sw = zeros(1, N_alpha);   Kp_L_sw = zeros(1, N_alpha);

    for i = 1:N_alpha
        [fy_r, ~, t_r] = brush_model(alphas(i), Fz_R, mu_R, Cfa_R, ap);
        [fy_l, ~, t_l] = brush_model(alphas(i), Fz_L, mu_L, Cfa_L, ap);
        Fy_R_sw(i) = fy_r;   Kp_R_sw(i) = fy_r * (e + t_r);
        Fy_L_sw(i) = fy_l;   Kp_L_sw(i) = fy_l * (e + t_l);
    end

    % Required Fy per wheel, split proportional to load
    Fy_total = m * ay_abs * (b_cg / L);
    denom    = max(Fz_R + Fz_L, 1);
    Fy_req_R = Fy_total * Fz_R / denom;
    Fy_req_L = Fy_total * Fz_L / denom;

    % Operating points
    idx_R = find_op(Fy_R_sw, Fy_req_R, N_alpha);
    idx_L = find_op(Fy_L_sw, Fy_req_L, N_alpha);

    M_kp_R = (Fz_R > 0) * Kp_R_sw(idx_R);
    M_kp_L = (Fz_L > 0) * Kp_L_sw(idx_L);

    M_kp_R_vec(k)  = M_kp_R;
    M_kp_L_vec(k)  = M_kp_L;
    alpha_R_vec(k) = alphas(idx_R);
    alpha_L_vec(k) = alphas(idx_L);

    % Both wheels always sum at rack. Sign follows ay: positive ay (right
    % turn) rotates pinion one direction, negative ay the other. This is
    % essential for fatigue analysis — mean torque and reversal counting
    % depend on signed values, not magnitudes.
    sign_ay = sign(ay);
    if sign_ay == 0, sign_ay = 1; end   % avoid sign(0)=0 ambiguity at ay=0
    F_rack_vec(k) = sign_ay * (M_kp_R + M_kp_L) / L_arm;
    T_SW_vec(k)   = F_rack_vec(k) * r_pin;
end


%% Sweep at constant G (steady-state)

ay_d    = abs(AY_DIAG_G) * g_SI; %lateral acc
dFz_d   = m * ay_d * h_cg * (b_cg / L) / (t_f); %load transfer
Fz_Rd   = Fz_static + dFz_d;   % right wheel normal load w/ load trans
Fz_Ld   = max(Fz_static - dFz_d, 0); % Left wheel normal load w/ load trans

mu_Rd = mu_y_fn(Fz_Rd); Cfa_Rd = Cfa_fn(Fz_Rd); % right wheel coeffs at given Fz
mu_Ld = mu_y_fn(Fz_Ld); Cfa_Ld = Cfa_fn(Fz_Ld); % left wheel coeffs at given Fz

Fy_Rd = zeros(1,N_alpha); Mz_Rd = zeros(1,N_alpha);
Kp_Rd = zeros(1,N_alpha);  T_Rd  = zeros(1,N_alpha);
Fy_Ld = zeros(1,N_alpha); Mz_Ld = zeros(1,N_alpha);
Kp_Ld = zeros(1,N_alpha);  T_Ld  = zeros(1,N_alpha);

for i = 1:N_alpha
    [fy_r, mz_r, tr] = brush_model(alphas(i), Fz_Rd, mu_Rd, Cfa_Rd, ap);
    [fy_l, mz_l, tl] = brush_model(alphas(i), Fz_Ld, mu_Ld, Cfa_Ld, ap);
    Fy_Rd(i) = fy_r; Mz_Rd(i) = mz_r; Kp_Rd(i) = fy_r*(e+tr); T_Rd(i) = tr*1000;
    Fy_Ld(i) = fy_l; Mz_Ld(i) = mz_l; Kp_Ld(i) = fy_l*(e+tl); T_Ld(i) = tl*1000;
end
Fy_total_d = m * ay_d * (b_cg / L);
denom_d    = max(Fz_Rd + Fz_Ld, 1);
idx_Rd = find_op(Fy_Rd, Fy_total_d * Fz_Rd / denom_d, N_alpha);
idx_Ld = find_op(Fy_Ld, Fy_total_d * Fz_Ld / denom_d, N_alpha);


%% PRINT SUMMARY

% Peak in either direction (max of |T_SW|), plus signed extremes
[T_SW_absmax, idx_peak] = max(abs(T_SW_vec));
T_SW_max_pos = max(T_SW_vec);
T_SW_min_neg = min(T_SW_vec);
T_SW_mean    = mean(T_SW_vec);                % signed mean
T_SW_mean_abs= mean(abs(T_SW_vec));           % mean magnitude
T_SW_rms     = sqrt(mean(T_SW_vec.^2));       % RMS
T_SW_std     = std(T_SW_vec);                 % standard deviation
F_rack_rms   = sqrt(mean(F_rack_vec.^2));
F_rack_absmax= max(abs(F_rack_vec));

fprintf('\n====== Results Summary ======\n');
fprintf('  Peak |T_SW|       = %.2f Nm  @ ay = %+.2f G\n', T_SW_absmax, ay_g_vec(idx_peak));
fprintf('  Max T_SW (right) = %+.2f Nm\n', T_SW_max_pos);
fprintf('  Min T_SW (left)  = %+.2f Nm\n', T_SW_min_neg);
fprintf('  Mean T_SW (signed) = %+.3f Nm     <- use this for fatigue mean\n', T_SW_mean);
fprintf('  Mean |T_SW|       = %.3f Nm\n',  T_SW_mean_abs);
fprintf('  RMS T_SW          = %.3f Nm     <- use this for fatigue amplitude\n', T_SW_rms);
fprintf('  Std T_SW          = %.3f Nm\n',  T_SW_std);
fprintf('  Peak |F_rack|     = %.0f N\n',   F_rack_absmax);
fprintf('  RMS F_rack        = %.0f N\n',   F_rack_rms);
fprintf('  Peak M_kp_right   = %.2f Nm\n',  max(M_kp_R_vec));
fprintf('  Peak M_kp_left    = %.2f Nm\n',  max(M_kp_L_vec));
if strcmp(INPUT_MODE, 'data')
    fprintf('  Mean |ay|         = %.3f G\n',   mean(abs(ay_g_vec)));
    fprintf('  Time > +1G        = %.1f%%\n', sum(ay_g_vec >  1) / N_pts * 100);
    fprintf('  Time < -1G        = %.1f%%\n', sum(ay_g_vec < -1) / N_pts * 100);
end

%% ══════════════════════════════════════════════════════════════════════════════
%% FIGURE 1 — MAIN: T_SW, F_RACK AND PER-WHEEL KINGPIN MOMENT VS AY
%% ══════════════════════════════════════════════════════════════════════════════
figure('Name', 'AR26 Pinion Torque vs Lateral Acceleration', ...
       'Position', [80 80 1050 460]);

subplot(1,2,1);
yyaxis left
plot(ay_g_vec, T_SW_vec, 'LineWidth', 4);
ylabel('Pinion torque T_{SW} (Nm)', 'FontSize', 15);
ylim([min(T_SW_vec)*1.15 - 0.01, max(T_SW_vec)*1.15 + 0.01]);
yyaxis right
plot(ay_g_vec, F_rack_vec, 'LineWidth', 1.5);
ylabel('Rack force F_{rack} (N)', 'FontSize', 15);
xlabel('Lateral Acceleration (g)', 'FontSize', 15);
title('T_{P} and F_{rack}', 'FontSize', 20);
yline(0, 'k-', 'LineWidth', 0.5, 'HandleVisibility', 'off');
if strcmp(INPUT_MODE, 'sweep')
    xline(0, 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');
end
grid on;
legend('T_{SW}', 'F_{rack}', 'Location', 'north', 'FontSize', 12);

subplot(1,2,2);
plot(ay_g_vec, M_kp_R_vec, 'LineWidth', 2, ...
    'DisplayName', 'Right wheel M_{sa}'); hold on;
plot(ay_g_vec, M_kp_L_vec, 'LineWidth', 2, ...
    'DisplayName', 'Left wheel M_{sa}');
plot(ay_g_vec, M_kp_R_vec + M_kp_L_vec, 'Color',[0.466, 0.674, 0.188], 'LineWidth', 1.5, ...
    'DisplayName', 'Total M_{sa} (both)');
xlim([-3 3])
if strcmp(INPUT_MODE, 'sweep')
    xline(0, 'k:', 'LineWidth', 1, 'HandleVisibility', 'off');
end
xlabel('Lateral Acceleration (g)', 'FontSize', 15);
ylabel('Steering axis moment M_{sa} (Nm)', 'FontSize', 15);
title({'Steering axis Moment per Wheel'}, 'FontSize', 20);
legend('Location', 'north');
grid on;
set(gcf, 'Units', 'centimeters', 'Position', [30, 20, 35, 11]); % Screen size
set(gcf, 'PaperUnits', 'centimeters', 'PaperPosition', [30, 20, 35, 21]); % Export size

sgtitle('Pinion Torque vs Lateral Acceleration', ...
    'FontSize', 25, 'FontWeight', 'bold');

%% ══════════════════════════════════════════════════════════════════════════════
%% FIGURE 2 — DATA MODE: time history + histogram
%% ══════════════════════════════════════════════════════════════════════════════
if strcmp(INPUT_MODE, 'data')

    % ── Figure 2a: T_SW time history + histogram ──────────────────────────────
    figure('Name', 'AR26 Steering Moment — Lap Data', 'Position', [80 620 1050 420]);

    subplot(1,3,1:2);
    yyaxis left
    plot(t_vec, T_SW_vec, 'b-', 'LineWidth', 1.0, 'DisplayName', 'T_{SW}'); hold on;
    % Mean and ±RMS reference bands
    yline(T_SW_mean, 'b-', 'LineWidth', 1.2, 'Alpha', 0.6, ...
        'Label', sprintf('Mean = %+.2f Nm', T_SW_mean), 'FontSize', 8, ...
        'HandleVisibility', 'off');
    yline( T_SW_rms,  'b:', 'LineWidth', 1, 'Alpha', 0.5, ...
        'Label', sprintf('+RMS = %.2f Nm', T_SW_rms), 'FontSize', 8, ...
        'HandleVisibility', 'off');
    yline(-T_SW_rms,  'b:', 'LineWidth', 1, 'Alpha', 0.5, ...
        'Label', sprintf('-RMS = %.2f Nm', -T_SW_rms), 'FontSize', 8, ...
        'HandleVisibility', 'off');
    ylabel('T_{SW} (Nm)');
    ylim([min(T_SW_vec)*1.2 - 0.01, max(T_SW_vec)*1.2 + 0.01]);
    yyaxis right
    plot(t_vec, ay_g_vec, 'Color', [0.5 0.5 0.5], 'LineWidth', 0.8, ...
        'DisplayName', 'a_y (signed)');
    yline(0, 'k:', 'LineWidth', 0.8, 'HandleVisibility', 'off');
    ylabel('a_y (G)');
    xlabel('Time (s)');
    title('Signed Steering Torque and Lateral Acceleration');
    legend('Location', 'northwest');
    grid on;

    subplot(1,3,3);
    histogram(T_SW_vec, 50, 'FaceColor', [0.25 0.4 0.7], 'EdgeColor', 'none');
    xlabel('T_{SW} (Nm)');
    ylabel('Count');
    title('Distribution of Signed Steering Torque');
    xline(0,            'k-',  'LineWidth', 1.0, 'HandleVisibility', 'off');
    xline(T_SW_mean,    'r--', 'LineWidth', 1.5, ...
        'Label', sprintf('Mean %+.2f', T_SW_mean), 'FontSize', 8);
    xline( T_SW_rms,    'm--', 'LineWidth', 1.5, ...
        'Label', sprintf('+RMS %.2f', T_SW_rms), 'FontSize', 8);
    xline(-T_SW_rms,    'm--', 'LineWidth', 1.5, ...
        'Label', sprintf('-RMS %.2f', -T_SW_rms), 'FontSize', 8);
    grid on;

    sgtitle('AR26 — Lap Data Steering Analysis  (FSG Endurance 2012)', ...
        'FontSize', 12, 'FontWeight', 'bold');

    % ── Figure 2b: Per-physical-wheel kingpin moment over time ────────────────
    figure('Name', 'AR26 Kingpin Moment per Wheel vs Time', ...
           'Position', [80 120 1100 520]);

    % Top two thirds: kingpin moments
    subplot(3,1,1:2);
    area(t_vec, M_kp_R_vec, 'FaceColor', [0.18 0.42 0.78], ...
        'FaceAlpha', 0.55, 'EdgeColor', 'none', 'DisplayName', 'Right wheel'); hold on;
    area(t_vec, M_kp_L_vec, 'FaceColor', [0.82 0.22 0.22], ...
        'FaceAlpha', 0.55, 'EdgeColor', 'none', 'DisplayName', 'Left wheel');
    plot(t_vec, M_kp_R_vec + M_kp_L_vec, 'k-', 'LineWidth', 1.2, ...
        'DisplayName', 'Total M_{kp}');
    [M_peak, i_peak] = max(M_kp_R_vec + M_kp_L_vec);
    plot(t_vec(i_peak), M_peak, 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 7, ...
        'HandleVisibility', 'off');
    text(t_vec(i_peak), M_peak * 1.06, sprintf('Peak: %.1f Nm', M_peak), ...
        'FontSize', 8, 'HorizontalAlignment', 'center');
    ylabel('Kingpin moment M_{kp} (Nm)');
    title(sprintf(['Per-Physical-Wheel Kingpin Moment — FSG Endurance 2012\n' ...
        'Right wheel outer at +a_y  |  Left wheel outer at -a_y']));
    legend('Location', 'northeast', 'FontSize', 9);
    grid on;
    xlim([t_vec(1) t_vec(end)]);

    % Bottom third: signed ay for context with fill to show direction
    subplot(3,1,3);
    fill([t_vec, fliplr(t_vec)], ...
         [max(ay_g_vec, 0), zeros(1, numel(t_vec))], ...
         [0.18 0.42 0.78], 'FaceAlpha', 0.4, 'EdgeColor', 'none', ...
         'DisplayName', 'Right (+)'); hold on;
    fill([t_vec, fliplr(t_vec)], ...
         [min(ay_g_vec, 0), zeros(1, numel(t_vec))], ...
         [0.82 0.22 0.22], 'FaceAlpha', 0.4, 'EdgeColor', 'none', ...
         'DisplayName', 'Left (-)');
    plot(t_vec, ay_g_vec, 'Color', [0.3 0.3 0.3], 'LineWidth', 0.7, ...
        'HandleVisibility', 'off');
    yline(0, 'k-', 'LineWidth', 0.5);
    ylabel('a_y (G)');
    xlabel('Time (s)');
    title('Lateral Acceleration (signed)');
    legend('Location', 'northeast', 'FontSize', 9);
    grid on;
    xlim([t_vec(1) t_vec(end)]);

    sgtitle(sprintf(['AR26 — Kingpin Moment over Lap  |  FSG Endurance 2012\n' ...
        'Peak |T_{SW}| = %.1f Nm  |  Mean T_{SW} = %+.2f Nm  |  RMS T_{SW} = %.2f Nm'], ...
        T_SW_absmax, T_SW_mean, T_SW_rms), ...
        'FontSize', 11, 'FontWeight', 'bold');

    % ── Figure 2c: M_kp scatter vs ay coloured by T_SW ───────────────────────
    figure('Name', 'AR26 Kingpin Moment vs Lateral Acceleration', ...
           'Position', [80 80 600 440]);

    scatter(ay_g_vec, M_kp_R_vec + M_kp_L_vec, 4, T_SW_vec, 'filled', ...
        'MarkerFaceAlpha', 0.4);
    colormap(gca, 'cool');
    cb = colorbar;
    cb.Label.String = 'T_{SW} (Nm)';
    xlabel('Lateral acceleration a_y (G)');
    ylabel('Total kingpin moment M_{kp} (Nm)');
    title({'Kingpin Moment vs Lateral Acceleration'; ...
           'colour = steering wheel torque T_{SW}'});
    xline(0, 'k:', 'LineWidth', 0.8);
    grid on;

    sgtitle('FSG Endurance 2012', 'FontSize', 11, 'FontWeight', 'bold');

    % ── Figure 2d: Track map coloured by |T_SW| (green=low, red=high) ────────
    if have_pos
        figure('Name', 'AR26 Track Map — Steering Torque Heatmap', ...
               'Position', [80 80 850 700]);

        % Use |T_SW| for the colour scale — "badness" is magnitude regardless
        % of direction. Switch to T_SW_vec directly if you want signed colour.
        c_data = T_SW_vec;

        % Build the coloured line using patch with per-vertex colours.
        % Trick: pass a NaN-terminated polyline as a single patch with the
        % CData mapped to each vertex. EdgeColor = 'interp' interpolates
        % colour along each edge between consecutive vertices.
        h = patch([x_pos, NaN], [y_pos, NaN], [c_data, NaN], ...
            'EdgeColor', 'interp', 'FaceColor', 'none', 'LineWidth', 3);

        % Diverging blue -> green -> red colormap, centred on zero
        % blue = max negative (hard left), green = zero (straight), red = max positive (hard right)
        clim_abs = max(abs(T_SW_vec)) * 1.02;    
        n_cmap = 256;
            cmap   = zeros(n_cmap, 3);
            for i = 1:n_cmap
                f = (i-1)/(n_cmap-1);
                if f < 0.5
    % muted blue [0.5, 0.5, 0.9] → muted green [0.5, 0.85, 0.5]
    t = f / 0.5;
    cmap(i,:) = [0.5 + t*0.0,  0.5 + t*0.35,  0.9 - t*0.40];
else
    % muted green [0.5, 0.85, 0.5] → muted red [0.9, 0.5, 0.5]
    t = (f - 0.5) / 0.5;
    cmap(i,:) = [0.5 + t*0.40,  0.85 - t*0.35,  0.5 + t*0.0];
end
            end
        colormap(gca, cmap);
        clim([-clim_abs, clim_abs]);

        cb = colorbar;
        cb.Label.String = '|T_{P}| (Nm)';
        cb.Label.FontSize = 15;
        cb.Label.FontName = 'Helvetica';

        axis equal;
        grid on;
        xlabel('x position (m)', 'FontSize', 15, 'FontName','Helvetica');
        ylabel('y position (m)', 'FontSize', 15, 'FontName','Helvetica');
        %title({'Track Map Coloured by Steering Torque Magnitude'; ...
        %       sprintf('Peak |T_{SW}| = %.2f Nm  |  Mean = %+.3f Nm  |  RMS = %.2f Nm', ...
        %               T_SW_absmax, T_SW_mean, T_SW_rms)});

        % Mark start/finish and peak torque location
        hold on;
        plot(x_pos(1), y_pos(1), 'ks', 'MarkerSize', 9, ...
            'MarkerFaceColor', [0.3 0.3 0.3], 'DisplayName', 'Start');
        [~, i_peak] = max(c_data);
        plot(x_pos(i_peak), y_pos(i_peak), 'kp', 'MarkerSize', 14, ...
            'MarkerFaceColor', [0.85 0.05 0.05], ...
            'DisplayName', sprintf('Peak %.1f Nm', c_data(i_peak)));
        legend('Location', 'northeast', 'FontSize', 12, 'FontName','Helvetica');
        hold off;

        sgtitle('FSA Endurance 2012', 'FontSize', 25, 'FontWeight', 'bold', 'FontName','Helvetica');
    end

end
set(gcf, 'Units', 'centimeters', 'Position', [30, 20, 25, 25]); % Screen size
set(gcf, 'PaperUnits', 'centimeters', 'PaperPosition', [0, 0, 15, 10]); % Export size
%saveas(gcf,'trackmapAus','svg');

%% ══════════════════════════════════════════════════════════════════════════════
%% FIGURE 3 — DIAGNOSTIC BRUSH SWEEP AT +AY_DIAG_G
%% ══════════════════════════════════════════════════════════════════════════════
figure('Name', sprintf('Brush Sweep @ +%.1fg  (right=outer)', AY_DIAG_G), ...
      'Position', [0 80 2560 560]);

subplot(1,3,1);
%figure
plot(alphas, Fy_Rd, 'LineWidth',3,   'DisplayName','Right (outer)'); hold on;
plot(alphas, Fy_Ld, 'LineWidth',2.5, 'DisplayName','Left (inner)');
xline(alphas(idx_Rd),'Color',[0, 0.4470, 0.7410],'LineStyle','--','LineWidth',1.5,'HandleVisibility','off');
xline(alphas(idx_Ld),'Color',[0.8500, 0.3250, 0.0980],'LineStyle','--','LineWidth',1.5,'HandleVisibility','off');
plot(alphas(idx_Rd),Fy_Rd(idx_Rd),'bo','MarkerFaceColor',[0, 0.4470, 0.7410],'MarkerSize',10,...
    'DisplayName',sprintf('Op. right (%.1f deg)',alphas(idx_Rd)));
plot(alphas(idx_Ld),Fy_Ld(idx_Ld),'ro','MarkerFaceColor',[0.8500, 0.3250, 0.0980],'MarkerSize',10,...
    'DisplayName',sprintf('Op. left  (%.1f deg)',alphas(idx_Ld)));
grid on; legend('Location','southeast','FontSize',12);
xlabel('\alpha (deg)', 'FontSize',15); ylabel('F_y (N)', 'FontSize',15);
title('Lateral Force', 'FontSize', 20);
%set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 15, 10]); % Screen size
%set(gcf, 'PaperUnits', 'centimeters', 'PaperPosition', [0, 0, 15, 10]); % Export size

subplot(1,3,2);
%figure
yyaxis left
plot(alphas,Mz_Rd,'Color',[0 0.447 0.741],'LineWidth',3,  'DisplayName','M_z right'); hold on;
plot(alphas,Mz_Ld,'Color',[0.85 0.325 0.098],'LineWidth',2.5,'DisplayName','M_z left');
ylabel('M_z (Nm)', 'FontSize',15);
yyaxis right
ax = gca; 
ax.YAxis(2).Color = [0.929, 0.694, 0.125];
plot(alphas,T_Rd,'Color',[0.929, 0.694, 0.125],'LineWidth',1.5,'DisplayName','t right (mm)');
plot(alphas,T_Ld,'Color',[0.466, 0.674, 0.188],'LineWidth',1.2,'DisplayName','t left  (mm)');
ylabel('Pneumatic trail t (mm)','Color',[0.929, 0.694, 0.125], 'FontSize',15);
grid on; legend('Location','northeast','FontSize',12);
xlabel('\alpha (deg)', 'FontSize',15);
title('Aligning Moment and Pneumatic Trail', 'FontSize',20);
%set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 15, 10]); % Screen size
%set(gcf, 'PaperUnits', 'centimeters', 'PaperPosition', [0, 0, 15, 10]); % Export size

subplot(1,3,3);
%figure
plot(alphas,Kp_Rd,'Color',[0 0.447 0.741],'LineWidth',3,  'DisplayName','M_{kp} right'); hold on;
plot(alphas,Kp_Ld,'Color',[0.85 0.325 0.098],'LineWidth',2.5,'DisplayName','M_{kp} left');
xline(alphas(idx_Rd),'Color',[0 0.447 0.741],'LineWidth',1.5,'HandleVisibility','off');
xline(alphas(idx_Ld),'Color',[0.85 0.325 0.098],'LineWidth',1.5,'HandleVisibility','off');
plot(alphas(idx_Rd),Kp_Rd(idx_Rd),'bo','MarkerFaceColor',[0 0.447 0.741],'MarkerSize',10,...
    'DisplayName',sprintf('Right: %.1f Nm',Kp_Rd(idx_Rd)));
plot(alphas(idx_Ld),Kp_Ld(idx_Ld),'ro','MarkerFaceColor',[0.85 0.325 0.098],'MarkerSize',10,...
    'DisplayName',sprintf('Left:  %.1f Nm',Kp_Ld(idx_Ld)));
grid on; legend('Location','southeast','FontSize',12);
xlabel('\alpha (deg)', 'FontSize',15); ylabel('M_{sa}  (Nm)', 'FontSize',15);
title('Steering Axis Moment','FontSize', 20);
set(gcf, 'Units', 'centimeters', 'Position', [0, 0, 45, 10]); % Screen size
set(gcf, 'PaperUnits', 'centimeters', 'PaperPosition', [0, 0, 45, 10]); % Export size

% subplot(2,2,4);
% T_SW_d   = (Kp_Rd(idx_Rd) + Kp_Ld(idx_Ld)) / L_arm * r_pin;
% F_rack_d = (Kp_Rd(idx_Rd) + Kp_Ld(idx_Ld)) / L_arm;
% cats = {'M_{kp} right','M_{kp} left','F_{rack}/10','T_{SW}'};
% vals = [Kp_Rd(idx_Rd), Kp_Ld(idx_Ld), F_rack_d/10, T_SW_d];
% bar(vals,'FaceColor',[0.25 0.35 0.6]);
% set(gca,'XTickLabel',cats,'XTickLabelRotation',20,'FontSize',9);
% ylabel('Nm  (or N/10 for F_{rack})');
% title(sprintf('Summary  T_{SW}=%.2f Nm  F_{rack}=%.0f N', T_SW_d, F_rack_d));
% grid on;

sgtitle('Brush Sweep (right=outer, left=inner)', ...
    'FontSize', 25, 'FontWeight', 'bold');

%% ══════════════════════════════════════════════════════════════════════════════
%% FIGURE 3 — BRUSH SWEEP + MOMENT PATH SUMMARY AT +AY_DIAG_G
%% ══════════════════════════════════════════════════════════════════════════════

c_blue   = [0,      0.4470, 0.7410];
c_orange = [0.8500, 0.3250, 0.0980];
c_green  = [0.466,  0.674,  0.188 ];
c_yellow = [0.929,  0.694,  0.125 ];

% Derived summary quantities at operating point
T_SW_d    = (Kp_Rd(idx_Rd) + Kp_Ld(idx_Ld)) / L_arm * r_pin;
F_rack_d  = (Kp_Rd(idx_Rd) + Kp_Ld(idx_Ld)) / L_arm;

figure('Name', sprintf('Brush Sweep + Summary @ %.1fg', AY_DIAG_G), ...
    'Position', [80 80 1400 440]);

% ── Panel 1: Lateral Force ────────────────────────────────────────────────────
subplot(1,3,1);
plot(alphas, Fy_Rd, 'Color', c_blue,   'LineWidth', 2.5, ...
    'DisplayName', 'Outer wheel'); hold on;
plot(alphas, Fy_Ld, 'Color', c_orange, 'LineWidth', 2.5, ...
    'DisplayName', 'Inner wheel');
xline(alphas(idx_Rd), 'Color', c_blue,   'LineStyle', '--', 'LineWidth', 1.5, ...
    'HandleVisibility', 'off');
xline(alphas(idx_Ld), 'Color', c_orange, 'LineStyle', '--', 'LineWidth', 1.5, ...
    'HandleVisibility', 'off');
plot(alphas(idx_Rd), Fy_Rd(idx_Rd), 'o', 'Color', c_blue, ...
    'MarkerFaceColor', c_blue,   'MarkerSize', 9, ...
    'DisplayName', sprintf('Outer: \\alpha = %.1f°', alphas(idx_Rd)));
plot(alphas(idx_Ld), Fy_Ld(idx_Ld), 'o', 'Color', c_orange, ...
    'MarkerFaceColor', c_orange, 'MarkerSize', 9, ...
    'DisplayName', sprintf('Inner: \\alpha = %.1f°', alphas(idx_Ld)));
grid on;
xlabel('\alpha (deg)', 'FontSize', 15);
ylabel('F_y (N)',      'FontSize', 15);
title('Lateral Force', 'FontSize', 20);
legend('Location', 'southeast', 'FontSize', 12);

% ── Panel 2: Kingpin Moment ───────────────────────────────────────────────────
subplot(1,3,2);
plot(alphas, Kp_Rd, 'Color', c_blue,   'LineWidth', 2.5, ...
    'DisplayName', 'Outer wheel'); hold on;
plot(alphas, Kp_Ld, 'Color', c_orange, 'LineWidth', 2.5, ...
    'DisplayName', 'Inner wheel');
xline(alphas(idx_Rd), 'Color', c_blue,   'LineStyle', '--', 'LineWidth', 1.5, ...
    'HandleVisibility', 'off');
xline(alphas(idx_Ld), 'Color', c_orange, 'LineStyle', '--', 'LineWidth', 1.5, ...
    'HandleVisibility', 'off');
plot(alphas(idx_Rd), Kp_Rd(idx_Rd), 'o', 'Color', c_blue, ...
    'MarkerFaceColor', c_blue,   'MarkerSize', 9, ...
    'DisplayName', sprintf('Outer: %.1f Nm', Kp_Rd(idx_Rd)));
plot(alphas(idx_Ld), Kp_Ld(idx_Ld), 'o', 'Color', c_orange, ...
    'MarkerFaceColor', c_orange, 'MarkerSize', 9, ...
    'DisplayName', sprintf('Inner: %.1f Nm', Kp_Ld(idx_Ld)));
grid on;
xlabel('\alpha (deg)', 'FontSize', 15);
ylabel('M_{kp} (Nm)', 'FontSize', 15);
title('Kingpin Moment', 'FontSize', 20);
legend('Location', 'southeast', 'FontSize', 12);

% ── Panel 3: Moment path summary bar chart ────────────────────────────────────
subplot(1,3,3);

% Values to display — all in Nm except F_rack shown as Nm-equivalent (F*r_pin)
bar_labels = {'M_{kp} outer', 'M_{kp} inner', 'F_{rack} \times r_{pin}'};
bar_values = [Kp_Rd(idx_Rd), Kp_Ld(idx_Ld), T_SW_d];
bar_colors = [c_blue; c_orange; c_green];

b = bar(bar_values, 'FaceColor', 'flat');
b.CData = bar_colors;
b.EdgeColor = 'none';

% Annotate each bar with its value
for i = 1:3
    text(i, bar_values(i) + 0.3, sprintf('%.1f Nm', bar_values(i)), ...
        'HorizontalAlignment', 'center', 'FontSize', 12, 'FontWeight', 'bold');
end

% Add F_rack as secondary annotation on the T_SW bar
text(3, bar_values(3)/2, sprintf('F_{rack} = %.0f N', F_rack_d), ...
    'HorizontalAlignment', 'center', 'FontSize', 11, 'Color', 'w', 'FontWeight', 'bold');

set(gca, 'XTickLabel', bar_labels, 'FontSize', 12, 'XTickLabelRotation', 10);
ylabel('Moment (Nm)', 'FontSize', 15);
title('Moment Path', 'FontSize', 20);
ylim([0, max(bar_values) * 1.25]);
grid on; box off;

sgtitle(sprintf('Brush Model — a_y = %.1f g   |   F_{rack} = %.0f N   |   T_{pinion} = %.2f Nm', ...
    AY_DIAG_G, F_rack_d, T_SW_d), 'FontSize', 25, 'FontWeight', 'bold');

exportgraphics(gcf, 'brush_sweep_1p5g.pdf', 'ContentType', 'vector');


F_tierod_outer = Kp_Rd(idx_Rd) / L_arm;
F_tierod_inner = Kp_Ld(idx_Ld) / L_arm;
F_rack_d       = F_tierod_outer + F_tierod_inner;

load("drysteering.mat")
otherTp = max(Kp_Rd)./i + max(Kp_Ld)./i
figure
plot(delta_deg2,otherTp)


%%

%% ══════════════════════════════════════════════════════════════════════════════
%% LOCAL FUNCTIONS
%% ══════════════════════════════════════════════════════════════════════════════

function [Fy, Mz, t] = brush_model(alpha_deg, Fz, mu, Cfa, ap_m)
% Brush tyre model, rigid carcass, x1.4 carcass compliance correction.
% alpha_deg: slip angle magnitude (deg). Returns Fy (N), Mz (Nm), t (m).
    if Fz <= 0 || mu <= 0
        Fy = 0; Mz = 0; t = 0; return;
    end
    sy    = tan(alpha_deg * pi / 180);
    theta = Cfa / (3 * mu * Fz);
    xi    = theta * abs(sy);
    if xi <= 1
        Fy  = mu * Fz * (3*xi - 3*xi^2 + xi^3);
        num = 1 - 3*xi + 3*xi^2 - xi^3;
        den = max(1 - xi + xi^2/3, 1e-9);
        t   = max((ap_m / 3) * (num / den) * 1.4, 0);
    else
        Fy = mu * Fz;
        t  = 0;
    end
    Mz = t * Fy;
end

function idx = find_op(Fy_sweep, Fy_req, N)
% First index where Fy >= min(Fy_req, 99.5% of peak). Falls back to peak.
    target = min(max(Fy_req, 0), max(Fy_sweep) * 0.995);
    idx    = find(Fy_sweep >= target, 1, 'first');
    if isempty(idx)
        [~, idx] = max(Fy_sweep);
    end
    idx = min(max(idx, 1), N);
end
