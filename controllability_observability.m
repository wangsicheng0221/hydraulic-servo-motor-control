%% Controllability and Observability Analysis for Hydraulic Servo Motor Control System
% This script analyzes the system controllability and observability properties
% for the hydraulic servo motor control system
%
% Key concepts:
% - Controllability: Ability to steer the system state to any desired state
% - Observability: Ability to determine the system state from output measurements
%
% Author: wangsicheng0221
% Date: 2026-01-06

clear; clc; close all;

%% System Parameters Definition
% Hydraulic servo motor system parameters
m = 10;           % Mass of load (kg)
b = 5;            % Viscous friction coefficient (N·s/m)
Kp = 2;           % Pressure gain (proportional coefficient)
Tm = 0.05;        % Motor time constant (s)
Ts = 0.01;        % Sampling time (s)
Qn = 50;          % Nominal flow rate (L/min)

%% Continuous-Time System State-Space Model
% State variables: x = [position; velocity; pressure]
% Control input: u = [spool displacement]
% Output: y = [position; velocity]

% State-space matrices for continuous-time system
Ac = [0      1      0;
      0     -b/m    Kp/m;
      0      0     -1/Tm];

Bc = [0;
      0;
      Kp/Tm];

Cc = [1  0  0;
      0  1  0];

Dc = [0];

% Create continuous-time system
sys_continuous = ss(Ac, Bc, Cc, Dc);

%% Discrete-Time System State-Space Model (Zero-Order Hold)
sys_discrete = c2d(sys_continuous, Ts, 'zoh');
Ad = sys_discrete.A;
Bd = sys_discrete.B;
Cd = sys_discrete.C;
Dd = sys_discrete.D;

%% ==================== CONTROLLABILITY ANALYSIS ====================

fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('CONTROLLABILITY ANALYSIS\n');
fprintf('%s\n', repmat('=', 1, 70));

% Continuous-Time Controllability
fprintf('\n--- Continuous-Time System ---\n');
P_continuous = ctrb(Ac, Bc);
rank_P_continuous = rank(P_continuous);
n_continuous = size(Ac, 1);

fprintf('Controllability Matrix P = [B, AB, A²B]\n');
fprintf('P = \n');
disp(P_continuous);

fprintf('Rank of Controllability Matrix: %d\n', rank_P_continuous);
fprintf('System Order: %d\n', n_continuous);

if rank_P_continuous == n_continuous
    fprintf('✓ System is CONTROLLABLE (rank = system order)\n');
    controllable_continuous = true;
else
    fprintf('✗ System is NOT CONTROLLABLE (rank < system order)\n');
    controllable_continuous = false;
end

% Controllable and Uncontrollable Subspaces
fprintf('\nEigenvalues of A:\n');
eig_A = eig(Ac);
disp(eig_A);

% Discrete-Time Controllability
fprintf('\n--- Discrete-Time System ---\n');
P_discrete = ctrb(Ad, Bd);
rank_P_discrete = rank(P_discrete);
n_discrete = size(Ad, 1);

fprintf('Controllability Matrix P = [B, AB, A²B, ...]\n');
fprintf('P = \n');
disp(P_discrete);

fprintf('Rank of Controllability Matrix: %d\n', rank_P_discrete);
fprintf('System Order: %d\n', n_discrete);

if rank_P_discrete == n_discrete
    fprintf('✓ System is CONTROLLABLE (rank = system order)\n');
    controllable_discrete = true;
else
    fprintf('✗ System is NOT CONTROLLABLE (rank < system order)\n');
    controllable_discrete = false;
end

%% ==================== OBSERVABILITY ANALYSIS ====================

fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('OBSERVABILITY ANALYSIS\n');
fprintf('%s\n', repmat('=', 1, 70));

% Continuous-Time Observability
fprintf('\n--- Continuous-Time System ---\n');
Q_continuous = obsv(Ac, Cc);
rank_Q_continuous = rank(Q_continuous);

fprintf('Observability Matrix Q = [C; CA; CA²]\n');
fprintf('Q = \n');
disp(Q_continuous);

fprintf('Rank of Observability Matrix: %d\n', rank_Q_continuous);
fprintf('System Order: %d\n', n_continuous);

if rank_Q_continuous == n_continuous
    fprintf('✓ System is OBSERVABLE (rank = system order)\n');
    observable_continuous = true;
else
    fprintf('✗ System is NOT OBSERVABLE (rank < system order)\n');
    observable_continuous = false;
end

% Discrete-Time Observability
fprintf('\n--- Discrete-Time System ---\n');
Q_discrete = obsv(Ad, Cd);
rank_Q_discrete = rank(Q_discrete);

fprintf('Observability Matrix Q = [C; CA; CA²]\n');
fprintf('Q = \n');
disp(Q_discrete);

fprintf('Rank of Observability Matrix: %d\n', rank_Q_discrete);
fprintf('System Order: %d\n', n_discrete);

if rank_Q_discrete == n_discrete
    fprintf('✓ System is OBSERVABLE (rank = system order)\n');
    observable_discrete = true;
else
    fprintf('✗ System is NOT OBSERVABLE (rank < system order)\n');
    observable_discrete = false;
end

%% ==================== CONTROLLABILITY & OBSERVABILITY DECOMPOSITION ====================

fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('SYSTEM DECOMPOSITION ANALYSIS\n');
fprintf('%s\n', repmat('=', 1, 70));

% Controllable and Observable Canonical Forms
fprintf('\n--- Controllable Canonical Form Decomposition ---\n');
try
    [Ac_canon, Tc] = ctrbf(Ac, Bc, Cc);
    fprintf('Controllable Canonical Form transformation successful\n');
    fprintf('Transformation Matrix Tc:\n');
    disp(Tc);
catch
    fprintf('Cannot perform controllable canonical decomposition\n');
end

fprintf('\n--- Observable Canonical Form Decomposition ---\n');
try
    [Ao_canon, To] = obsvf(Ac, Cc);
    fprintf('Observable Canonical Form transformation successful\n');
    fprintf('Transformation Matrix To:\n');
    disp(To);
catch
    fprintf('Cannot perform observable canonical decomposition\n');
end

%% ==================== POLE-ZERO ANALYSIS ====================

fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('POLE-ZERO AND STABILITY ANALYSIS\n');
fprintf('%s\n', repmat('=', 1, 70));

% Transfer function analysis
fprintf('\n--- Continuous-Time Transfer Function ---\n');
[num, den] = ss2tf(Ac, Bc, Cc, Dc);
TF_continuous = tf(num, den);
fprintf('Transfer Function:\n');
disp(TF_continuous);

% Poles and Zeros
fprintf('\nPoles (eigenvalues of A):\n');
poles_continuous = pole(TF_continuous);
disp(poles_continuous);

fprintf('\nZeros:\n');
zeros_continuous = zero(TF_continuous);
if isempty(zeros_continuous)
    fprintf('No transmission zeros\n');
else
    disp(zeros_continuous);
end

% Hidden modes (uncontrollable/unobservable modes)
fprintf('\n--- Hidden Modes Analysis ---\n');
pole_TF = poles_continuous;
eig_A = eig(Ac);

fprintf('Eigenvalues of A: \n');
disp(eig_A);
fprintf('\nPoles from Transfer Function: \n');
disp(pole_TF);

hidden_modes = setdiff(eig_A, pole_TF);
if isempty(hidden_modes)
    fprintf('No hidden modes - System is both controllable and observable\n');
else
    fprintf('Hidden modes detected (uncontrollable or unobservable):\n');
    disp(hidden_modes);
end

%% ==================== GRAMIAN ANALYSIS ====================

fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('GRAMIAN ANALYSIS\n');
fprintf('%s\n', repmat('=', 1, 70));

% Controllability Gramian (continuous-time)
fprintf('\n--- Controllability Gramian (Continuous) ---\n');
try
    Wc = gram(sys_continuous, 'c');
    fprintf('Controllability Gramian Wc:\n');
    disp(Wc);
    
    % Eigenvalues of controllability gramian
    eig_Wc = eig(Wc);
    fprintf('\nEigenvalues of Wc:\n');
    disp(eig_Wc);
    
    % Condition number
    cond_Wc = cond(Wc);
    fprintf('\nCondition Number of Wc: %.4e\n', cond_Wc);
    if cond_Wc < 1e10
        fprintf('Good numerical conditioning for controllability\n');
    else
        fprintf('Poor numerical conditioning - may cause numerical issues\n');
    end
catch
    fprintf('Cannot compute controllability gramian\n');
end

% Observability Gramian (continuous-time)
fprintf('\n--- Observability Gramian (Continuous) ---\n');
try
    Wo = gram(sys_continuous, 'o');
    fprintf('Observability Gramian Wo:\n');
    disp(Wo);
    
    % Eigenvalues of observability gramian
    eig_Wo = eig(Wo);
    fprintf('\nEigenvalues of Wo:\n');
    disp(eig_Wo);
    
    % Condition number
    cond_Wo = cond(Wo);
    fprintf('\nCondition Number of Wo: %.4e\n', cond_Wo);
    if cond_Wo < 1e10
        fprintf('Good numerical conditioning for observability\n');
    else
        fprintf('Poor numerical conditioning - may cause numerical issues\n');
    end
catch
    fprintf('Cannot compute observability gramian\n');
end

%% ==================== VISUALIZATION ====================

figure('Position', [100 100 1200 800]);

% 1. Pole-Zero Map
subplot(2, 3, 1);
if ~isempty(poles_continuous) && ~isempty(zeros_continuous)
    pzmap(sys_continuous);
else
    pzmap(sys_continuous);
end
grid on;
title('Pole-Zero Map (Continuous-Time)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Real Axis');
ylabel('Imaginary Axis');

% 2. Step Response
subplot(2, 3, 2);
step(sys_continuous);
grid on;
title('Step Response', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time (s)');
ylabel('Output');

% 3. Bode Magnitude Plot
subplot(2, 3, 3);
bode(TF_continuous);
grid on;
title('Bode Plot (Magnitude)', 'FontSize', 12, 'FontWeight', 'bold');

% 4. Controllability Matrix Rank
subplot(2, 3, 4);
bar([1, 2], [rank_P_continuous, n_continuous], 'FaceColor', 'cyan', 'EdgeColor', 'black');
set(gca, 'XTickLabel', {'Rank(P)', 'System Order'});
title('Controllability Matrix Analysis', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Dimension');
grid on;
ylim([0, n_continuous + 1]);

% 5. Observability Matrix Rank
subplot(2, 3, 5);
bar([1, 2], [rank_Q_continuous, n_continuous], 'FaceColor', 'lightgreen', 'EdgeColor', 'black');
set(gca, 'XTickLabel', {'Rank(Q)', 'System Order'});
title('Observability Matrix Analysis', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Dimension');
grid on;
ylim([0, n_continuous + 1]);

% 6. Eigenvalue Distribution
subplot(2, 3, 6);
scatter(real(eig_A), imag(eig_A), 100, 'filled', 'red');
hold on;
circle_theta = linspace(0, 2*pi, 100);
plot(cos(circle_theta), sin(circle_theta), 'k--', 'LineWidth', 1.5);
grid on;
xlabel('Real Part');
ylabel('Imaginary Part');
title('Eigenvalue Distribution', 'FontSize', 12, 'FontWeight', 'bold');
axis equal;
xlim([-2, 0.5]);
ylim([-2, 2]);

sgtitle('Controllability and Observability Analysis Summary', 'FontSize', 14, 'FontWeight', 'bold');

%% ==================== SUMMARY REPORT ====================

fprintf('\n%s\n', repmat('=', 1, 70));
fprintf('SUMMARY REPORT\n');
fprintf('%s\n', repmat('=', 1, 70));

fprintf('\nContinuous-Time System:\n');
fprintf('  Controllable: %s\n', mat2str(controllable_continuous));
fprintf('  Observable: %s\n', mat2str(observable_continuous));
fprintf('  Both: %s\n', mat2str(controllable_continuous && observable_continuous));

fprintf('\nDiscrete-Time System:\n');
fprintf('  Controllable: %s\n', mat2str(controllable_discrete));
fprintf('  Observable: %s\n', mat2str(observable_discrete));
fprintf('  Both: %s\n', mat2str(controllable_discrete && observable_discrete));

fprintf('\nSystem Stability:\n');
if all(real(eig_A) < 0)
    fprintf('  Status: STABLE (all poles in left half-plane)\n');
else
    fprintf('  Status: UNSTABLE (poles in right half-plane detected)\n');
end

fprintf('\n%s\n\n', repmat('=', 1, 70));

%% ==================== FUNCTIONS ====================

function [Ac_canon, Tc] = ctrbf(A, B, C)
    % Controllable Canonical Form Transformation
    % Transforms system to controllable canonical form
    n = size(A, 1);
    P = ctrb(A, B);
    
    % Compute transformation matrix from Hankel matrix
    [~, S, V] = svd(P');
    Tc = V(:, 1:n)';
    
    Ac_canon = Tc * A / Tc;
end

function [Ao_canon, To] = obsvf(A, C)
    % Observable Canonical Form Transformation
    % Transforms system to observable canonical form
    n = size(A, 1);
    Q = obsv(A, C);
    
    % Compute transformation matrix
    [U, S, ~] = svd(Q);
    To = U(:, 1:n)';
    
    Ao_canon = To * A / To;
end
