%% Step Response Analysis for Hydraulic Servo Motor Control
% This script performs zero-state and zero-input response analysis
% for the hydraulic servo motor control system
%
% Author: wangsicheng0221
% Date: 2026-01-06
% Description: Analyzes the system response to step inputs under different
%              initial conditions (zero-state and zero-input)

clear all; close all; clc;

%% System Parameters Definition
% Define the hydraulic servo motor system parameters

% Motor and load parameters
J = 0.5;        % Moment of inertia (kg*m^2)
b = 0.1;        % Viscous friction coefficient (N*s/rad)
K_m = 10;       % Motor torque constant (N*m/A)
K_b = 10;       % Back-EMF constant (V*s/rad)
R = 1;          % Armature resistance (Ohm)
L = 0.01;       % Armature inductance (H)

% Hydraulic system parameters
K_h = 5;        % Hydraulic valve gain (m^3/s/V)
C_t = 0.001;    % Total compliance (m^3/Pa)
P_s = 210e5;    % Supply pressure (Pa)

%% State-Space Model for Servo System
% Create state-space representation: dx/dt = A*x + B*u, y = C*x + D*u

% State vector: [omega; i; p_l] 
% where omega = angular velocity, i = current, p_l = load pressure

A = [-b/J,  K_m/J,    0;
     -K_b/L, -R/L,    0;
     0,      0,     -1/(C_t)];

B = [0; 1/L; K_h/(C_t)];

C = [1, 0, 0];  % Output: angular velocity

D = 0;

% Create the continuous-time system
sys = ss(A, B, C, D);

%% Zero-State Response (Initial Conditions = 0)
% System response with zero initial conditions and step input

fprintf('=== ZERO-STATE RESPONSE ANALYSIS ===\n\n');
fprintf('System Order: %d\n', length(A));
fprintf('Number of Inputs: %d\n', size(B, 2));
fprintf('Number of Outputs: %d\n\n', size(C, 1));

% Step input
t_sim = [0:0.001:5];
u_step = ones(size(t_sim));

% Zero-state response
x0_zero = [0; 0; 0];  % Zero initial conditions
[y_zero_state, t_zero, x_zero] = lsim(sys, u_step, t_sim, x0_zero);

fprintf('Zero-State Response:\n');
fprintf('  Initial conditions: x(0) = [0, 0, 0]^T\n');
fprintf('  Input: Step function u(t) = 1 for t >= 0\n');
fprintf('  Steady-state output: %.4f rad/s\n', y_zero_state(end));

%% Zero-Input Response (Non-zero Initial Conditions, Input = 0)
% System response with non-zero initial conditions and zero input

fprintf('\n=== ZERO-INPUT RESPONSE ANALYSIS ===\n\n');

% Define non-zero initial conditions
x0_nonzero = [10; 2; 50e5];  % [omega_0, i_0, p_l0]
u_zero = zeros(size(t_sim));

% Zero-input response
[y_zero_input, t_input, x_input] = lsim(sys, u_zero, t_sim, x0_nonzero);

fprintf('Zero-Input Response:\n');
fprintf('  Initial conditions: x(0) = [%.2f, %.2f, %.2e]^T\n', ...
        x0_nonzero(1), x0_nonzero(2), x0_nonzero(3));
fprintf('  Input: u(t) = 0 for all t\n');
fprintf('  Steady-state output: %.6f rad/s\n', y_zero_input(end));

%% Total Response (Superposition)
% Total response = zero-state response + zero-input response

y_total = y_zero_state + y_zero_input;

fprintf('\n=== TOTAL RESPONSE (SUPERPOSITION) ===\n\n');
fprintf('Total Response = Zero-State Response + Zero-Input Response\n');
fprintf('  Initial conditions: x(0) = [%.2f, %.2f, %.2e]^T\n', ...
        x0_nonzero(1), x0_nonzero(2), x0_nonzero(3));
fprintf('  Input: Step function u(t) = 1 for t >= 0\n');
fprintf('  Steady-state output: %.4f rad/s\n', y_total(end));

%% Performance Metrics
% Calculate and display performance metrics

% Zero-state response metrics
[peak_zs, peak_idx_zs] = max(y_zero_state);
overshoot_zs = (peak_zs - y_zero_state(end)) / y_zero_state(end) * 100;
settling_time_zs = find(abs(y_zero_state - y_zero_state(end)) <= 0.02*y_zero_state(end), 1) * 0.001;

fprintf('\n=== PERFORMANCE METRICS ===\n\n');
fprintf('Zero-State Response Metrics:\n');
fprintf('  Peak response: %.4f rad/s at t = %.3f s\n', peak_zs, t_zero(peak_idx_zs));
fprintf('  Overshoot: %.2f %%\n', overshoot_zs);
fprintf('  Settling time (2%%): %.3f s\n', settling_time_zs);

% Zero-input response metrics
fprintf('\nZero-Input Response Metrics:\n');
fprintf('  Initial value: %.4f rad/s\n', y_zero_input(1));
fprintf('  Final value: %.6f rad/s\n', y_zero_input(end));
fprintf('  Decay behavior: Exponential decay\n');

%% Eigenvalue Analysis
% Analyze system stability through eigenvalues

eigenvalues = eig(A);
fprintf('\n=== SYSTEM STABILITY ANALYSIS ===\n\n');
fprintf('Eigenvalues of the system matrix A:\n');
for i = 1:length(eigenvalues)
    fprintf('  λ%d = %.4f + j%.4f\n', i, real(eigenvalues(i)), imag(eigenvalues(i)));
end

if all(real(eigenvalues) < 0)
    fprintf('\nSystem Status: STABLE (all eigenvalues have negative real parts)\n');
else
    fprintf('\nSystem Status: UNSTABLE (at least one eigenvalue has non-negative real part)\n');
end

%% Plotting
figure('Position', [100, 100, 1200, 800]);

% Subplot 1: Zero-State Response
subplot(2,3,1);
plot(t_zero, y_zero_state, 'b', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Zero-State Response (x(0) = 0, u(t) = 1)');
legend('y_{zs}(t)');

% Subplot 2: Zero-Input Response
subplot(2,3,2);
plot(t_input, y_zero_input, 'r', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Zero-Input Response (x(0) ≠ 0, u(t) = 0)');
legend('y_{zi}(t)');

% Subplot 3: Total Response
subplot(2,3,3);
plot(t_zero, y_total, 'g', 'LineWidth', 2);
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Total Response (Superposition)');
legend('y_{total}(t)');

% Subplot 4: Comparison of Responses
subplot(2,3,4);
hold on;
plot(t_zero, y_zero_state, 'b', 'LineWidth', 2, 'DisplayName', 'Zero-State');
plot(t_input, y_zero_input, 'r', 'LineWidth', 2, 'DisplayName', 'Zero-Input');
plot(t_zero, y_total, 'g--', 'LineWidth', 2, 'DisplayName', 'Total');
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('Response Comparison');
legend('Location', 'best');

% Subplot 5: State Variables - Angular Velocity
subplot(2,3,5);
hold on;
plot(t_zero, x_zero(:,1), 'b', 'LineWidth', 2, 'DisplayName', 'Zero-State');
plot(t_input, x_input(:,1), 'r', 'LineWidth', 2, 'DisplayName', 'Zero-Input');
grid on;
xlabel('Time (s)');
ylabel('Angular Velocity (rad/s)');
title('State x₁: Angular Velocity');
legend('Location', 'best');

% Subplot 6: State Variables - Current
subplot(2,3,6);
hold on;
plot(t_zero, x_zero(:,2), 'b', 'LineWidth', 2, 'DisplayName', 'Zero-State');
plot(t_input, x_input(:,2), 'r', 'LineWidth', 2, 'DisplayName', 'Zero-Input');
grid on;
xlabel('Time (s)');
ylabel('Current (A)');
title('State x₂: Armature Current');
legend('Location', 'best');

sgtitle('Hydraulic Servo Motor Control - Step Response Analysis');

%% Export Results Summary
fprintf('\n=== ANALYSIS COMPLETE ===\n');
fprintf('Generated plots showing zero-state, zero-input, and total responses.\n');
fprintf('System performance characteristics have been analyzed and documented.\n');
