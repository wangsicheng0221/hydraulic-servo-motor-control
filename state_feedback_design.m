%% State Feedback Design and Full-Dimensional State Observer Design
% This script implements state feedback control and a full-dimensional state observer
% for the hydraulic servo motor control system
% 
% Author: wangsicheng0221
% Date: 2026-01-06
% Description: State feedback pole placement and Luenberger observer design

clear all; close all; clc;

%% System Parameters
% Hydraulic servo motor system parameters
% State-space representation: dx/dt = Ax + Bu, y = Cx + Du

% Motor characteristics
J = 0.05;           % Moment of inertia (kg*m^2)
b = 0.1;            % Viscous friction coefficient (N*m*s/rad)
Kp = 10;            % Proportional valve gain (m^3/s/V)
Kq = 100;           % Flow gain coefficient
tau_v = 0.1;        % Valve time constant (s)

% Pressure drop and flow characteristics
Vp = 0.01;          % Pump displacement (m^3/rev)
Pl = 0.5e-3;        % Load pressure (Pa)
beta = 1.8e9;       % Fluid bulk modulus (Pa)
Vh = 0.02;          % Hydraulic system volume (m^3)

% Transfer function from valve input to motor angular velocity
% Simplified second-order system representation
A = [0    1    0;
     0    -b/J  Kq/J;
     0    0    -1/tau_v];

B = [0;
     0;
     Kp/tau_v];

C = [1  0  0];       % Output: motor position
D = 0;

disp('=== System State-Space Representation ===');
disp('State vector x = [position; velocity; valve_input]');
fprintf('System order: %d\n', size(A,1));
disp('Matrix A:'); disp(A);
disp('Matrix B:'); disp(B);
disp('Matrix C:'); disp(C);

%% System Analysis
% Check controllability and observability
Co = ctrb(A, B);           % Controllability matrix
Obs = obsv(A, C);          % Observability matrix

rank_Co = rank(Co);
rank_Obs = rank(Obs);
n = size(A, 1);

fprintf('\n=== Controllability and Observability Analysis ===\n');
fprintf('System order: %d\n', n);
fprintf('Controllability matrix rank: %d (Expected: %d)\n', rank_Co, n);
fprintf('Observability matrix rank: %d (Expected: %d)\n', rank_Obs, n);

if rank_Co == n
    disp('✓ System is CONTROLLABLE - state feedback design is feasible');
else
    disp('✗ System is NOT controllable - state feedback may not work');
end

if rank_Obs == n
    disp('✓ System is OBSERVABLE - full-dimensional observer design is feasible');
else
    disp('✗ System is NOT observable - observer may not work');
end

%% State Feedback Design (Pole Placement)
% Design state feedback controller: u = -Kx + r
% Place closed-loop poles at desired locations

fprintf('\n=== State Feedback Control Design ===\n');

% Desired pole locations for servo system
% Using multiple approaches for comparison
p_natural = sqrt(10);       % Natural frequency
zeta = 0.7;                 % Damping ratio

% Desired closed-loop poles
% p1: Real pole for first order dynamics
% p2, p3: Complex conjugate poles for second order dynamics
p1 = -5;
p2 = -p_natural*zeta + 1j*p_natural*sqrt(1-zeta^2);
p3 = conj(p2);

desired_poles = [p1; p2; p3];
fprintf('Desired closed-loop poles:\n');
disp(desired_poles);

% Compute state feedback gain using pole placement
K = place(A, B, desired_poles);

fprintf('\nState Feedback Gain Matrix K:\n');
disp(K);

% Closed-loop system
Acl = A - B*K;
Bcl = B;
Ccl = C;
Dcl = D;

fprintf('\nClosed-loop pole locations:\n');
eig_cl = eig(Acl);
disp(eig_cl);

%% Full-Dimensional State Observer (Luenberger Observer)
% Design observer: dz/dt = Az + Bu + L(y - Cz)
% where z is the observed state, L is the observer gain

fprintf('\n=== Full-Dimensional State Observer Design ===\n');

% Desired observer pole locations (faster than closed-loop poles)
% Observer poles should be faster (more negative) than controller poles
p_obs1 = -15;
p_obs2 = -10 + 1j*8;
p_obs3 = conj(p_obs2);

desired_obs_poles = [p_obs1; p_obs2; p_obs3];
fprintf('Desired observer poles:\n');
disp(desired_obs_poles);

% Compute observer gain using pole placement
% For observer: dz/dt = (A - LC)z + Bu + Ly
% We use place(A', C', poles)' to find L
L = place(A', C', desired_obs_poles)';

fprintf('\nObserver Gain Matrix L:\n');
disp(L);

% Closed-loop observer system
A_obs = A - L*C;
B_obs = [B, L];

fprintf('\nObserver pole locations:\n');
eig_obs = eig(A_obs);
disp(eig_obs);

%% Combined System: State Feedback + Observer
% Overall augmented system with observer and feedback
% x_dot = Ax + B(-K*z)
% z_dot = (A - LC)z + Bu + L(y)
% y = Cx

fprintf('\n=== Combined System (State Feedback + Observer) ===\n');

% Augmented system matrix (controller + observer)
A_aug = [A, -B*K;
         L*C, A - L*C - B*K];

B_aug = [B;
         B];

C_aug = [C, zeros(1, 3)];

fprintf('Augmented system order: %d\n', size(A_aug, 1));
disp('Augmented system eigenvalues:');
eig_aug = eig(A_aug);
disp(eig_aug);

%% Transient Response Analysis
fprintf('\n=== Transient Response Analysis ===\n');

% Step response of state feedback system
t = 0:0.01:5;
[y_sf, t_sf, x_sf] = step(ss(Acl, ones(3,1), Ccl, Dcl), t);

% Step response of system with observer
% Using full augmented system
sys_combined = ss(A_aug, [ones(3,1); zeros(3,1)], C_aug, 0);
[y_combined, t_combined, x_combined] = step(sys_combined, t);

figure('Name', 'State Feedback and Observer Response');
subplot(2,1,1);
plot(t_sf, y_sf, 'b-', 'LineWidth', 2);
hold on;
plot(t_combined, y_combined, 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Position Output');
title('Step Response: State Feedback vs Combined System (with Observer)');
legend('State Feedback', 'State Feedback + Observer');
grid on;

% State trajectories
subplot(2,1,2);
plot(t_sf, x_sf(:,1), 'b-', 'LineWidth', 1.5, 'DisplayName', 'Position');
hold on;
plot(t_sf, x_sf(:,2), 'g-', 'LineWidth', 1.5, 'DisplayName', 'Velocity');
plot(t_sf, x_sf(:,3), 'r-', 'LineWidth', 1.5, 'DisplayName', 'Valve Input');
xlabel('Time (s)');
ylabel('State Value');
title('State Trajectories with State Feedback Control');
legend;
grid on;

%% Frequency Response Analysis
fprintf('\n=== Frequency Response Analysis ===\n');

% Open-loop system
sys_open = ss(A, B, C, D);

% Closed-loop system with state feedback
sys_closed_sf = ss(Acl, ones(3,1), Ccl, Dcl);

% Frequency range
w = logspace(-2, 2, 500);

% Bode plots
figure('Name', 'Frequency Response Analysis');
subplot(2,1,1);
[mag_open, phase_open] = bode(sys_open, w);
semilogx(w, 20*log10(squeeze(mag_open)), 'b-', 'LineWidth', 2);
hold on;
[mag_closed, phase_closed] = bode(sys_closed_sf, w);
semilogx(w, 20*log10(squeeze(mag_closed)), 'r-', 'LineWidth', 2);
xlabel('Frequency (rad/s)');
ylabel('Magnitude (dB)');
title('Magnitude Response: Open-loop vs Closed-loop');
legend('Open-loop', 'Closed-loop (State Feedback)');
grid on;

subplot(2,1,2);
semilogx(w, squeeze(phase_open), 'b-', 'LineWidth', 2);
hold on;
semilogx(w, squeeze(phase_closed), 'r-', 'LineWidth', 2);
xlabel('Frequency (rad/s)');
ylabel('Phase (degrees)');
title('Phase Response: Open-loop vs Closed-loop');
legend('Open-loop', 'Closed-loop (State Feedback)');
grid on;

%% Performance Metrics
fprintf('\n=== Performance Metrics ===\n');

% Step response characteristics
info_sf = stepinfo(y_sf, t_sf);
fprintf('State Feedback System:\n');
fprintf('  Rise time: %.4f s\n', info_sf.RiseTime);
fprintf('  Settling time: %.4f s\n', info_sf.SettlingTime);
fprintf('  Overshoot: %.2f %%\n', info_sf.Overshoot);
fprintf('  Steady-state value: %.4f\n', y_sf(end));

info_combined = stepinfo(y_combined, t_combined);
fprintf('\nCombined System (State Feedback + Observer):\n');
fprintf('  Rise time: %.4f s\n', info_combined.RiseTime);
fprintf('  Settling time: %.4f s\n', info_combined.SettlingTime);
fprintf('  Overshoot: %.2f %%\n', info_combined.Overshoot);
fprintf('  Steady-state value: %.4f\n', y_combined(end));

%% Robustness Analysis
fprintf('\n=== Robustness Analysis ===\n');

% Sensitivity function S = (I + GK)^(-1)
% Complementary sensitivity T = GK(I + GK)^(-1)

% For state feedback system
G = sys_open;
K_tf = tf(-K(1), [1]);  % Simplified transfer function form

% Gain and phase margins
[gm, pm, wgc, wpc] = margin(G);
fprintf('Open-loop margins:\n');
fprintf('  Gain margin: %.2f dB\n', 20*log10(gm));
fprintf('  Phase margin: %.2f deg\n', pm);
fprintf('  Gain crossover frequency: %.4f rad/s\n', wgc);
fprintf('  Phase crossover frequency: %.4f rad/s\n', wpc);

% Closed-loop system margins
[gm_cl, pm_cl] = margin(Acl, B, Ccl, D);
fprintf('\nClosed-loop margins:\n');
fprintf('  Gain margin: %.2f dB\n', 20*log10(gm_cl));
fprintf('  Phase margin: %.2f deg\n', pm_cl);

%% Design Summary
fprintf('\n=== DESIGN SUMMARY ===\n');
fprintf('State Feedback Gain K:\n');
fprintf('  K = [%.4f, %.4f, %.4f]\n', K(1), K(2), K(3));

fprintf('\nObserver Gain L:\n');
fprintf('  L = [%.4f; %.4f; %.4f]\n', L(1), L(2), L(3));

fprintf('\nController Poles (Desired vs Actual):\n');
for i = 1:length(desired_poles)
    if imag(desired_poles(i)) == 0
        fprintf('  Pole %d: Desired = %.4f, Actual = %.4f\n', i, desired_poles(i), eig_cl(i));
    else
        fprintf('  Pole %d: Desired = %.4f ± j%.4f, Actual = %.4f ± j%.4f\n', i, ...
            real(desired_poles(i)), abs(imag(desired_poles(i))), ...
            real(eig_cl(i)), abs(imag(eig_cl(i))));
    end
end

fprintf('\nObserver Poles (Desired vs Actual):\n');
for i = 1:length(desired_obs_poles)
    if imag(desired_obs_poles(i)) == 0
        fprintf('  Pole %d: Desired = %.4f, Actual = %.4f\n', i, desired_obs_poles(i), eig_obs(i));
    else
        fprintf('  Pole %d: Desired = %.4f ± j%.4f, Actual = %.4f ± j%.4f\n', i, ...
            real(desired_obs_poles(i)), abs(imag(desired_obs_poles(i))), ...
            real(eig_obs(i)), abs(imag(eig_obs(i))));
    end
end

%% Export Results
fprintf('\n=== Results Saved ===\n');
save('state_feedback_observer_results.mat', 'A', 'B', 'C', 'K', 'L', ...
    'Acl', 'A_obs', 'desired_poles', 'desired_obs_poles', ...
    'eig_cl', 'eig_obs', 'y_sf', 't_sf', 'y_combined', 't_combined');
disp('Results exported to: state_feedback_observer_results.mat');

% Create a summary figure
figure('Name', 'Control System Design Summary');
sgtitle('State Feedback Design and Full-Dimensional Observer');

% Pole-zero plot
subplot(2,2,1);
pzmap(sys_open);
hold on;
plot(real(desired_poles), imag(desired_poles), 'rx', 'MarkerSize', 10, 'LineWidth', 2);
plot(real(desired_obs_poles), imag(desired_obs_poles), 'bs', 'MarkerSize', 8, 'LineWidth', 2);
title('Pole Placement');
legend('Open-loop poles/zeros', 'Controller pole targets', 'Observer pole targets');
grid on;

% Step response comparison
subplot(2,2,2);
plot(t_sf, y_sf, 'b-', 'LineWidth', 2);
hold on;
plot(t_combined, y_combined, 'r--', 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Output');
title('Step Response');
legend('State Feedback', 'State Feedback + Observer');
grid on;

% Gain matrices
subplot(2,2,3);
text(0.1, 0.8, sprintf('State Feedback Gain K:\n[%.4f  %.4f  %.4f]', K(1), K(2), K(3)), ...
    'FontSize', 10, 'FontName', 'Monospace');
text(0.1, 0.5, sprintf('Observer Gain L:\n[%.4f  %.4f  %.4f]^T', L(1), L(2), L(3)), ...
    'FontSize', 10, 'FontName', 'Monospace');
axis off;
title('Design Gains');

% Performance indicators
subplot(2,2,4);
perf_text = sprintf(['State Feedback Performance:\n' ...
    'Rise Time: %.4f s\n' ...
    'Settling Time: %.4f s\n' ...
    'Overshoot: %.2f %%\n\n' ...
    'System Controllable: %s\n' ...
    'System Observable: %s'], ...
    info_sf.RiseTime, info_sf.SettlingTime, info_sf.Overshoot, ...
    iif(rank_Co == n, 'Yes', 'No'), ...
    iif(rank_Obs == n, 'Yes', 'No'));
text(0.1, 0.5, perf_text, 'FontSize', 10, 'FontName', 'Monospace');
axis off;
title('Performance Metrics');

function out = iif(condition, true_val, false_val)
    if condition
        out = true_val;
    else
        out = false_val;
    end
end
