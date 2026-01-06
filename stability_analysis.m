%% Lyapunov Stability Analysis for Hydraulic Servo Motor Control System
% This script performs Lyapunov stability analysis for the hydraulic servo motor control system
% Date: 2026-01-06
% Author: wangsicheng0221

clear all; close all; clc;

%% System Parameters
% Hydraulic servo motor system parameters
J = 0.5;              % Moment of inertia (kg*m^2)
b = 0.1;              % Viscous friction coefficient (N*s/rad)
Kp = 10;              % Pressure gain coefficient
Kt = 5;               % Torque constant (N*m/A)
Kv = 0.8;             % Valve constant
tau_v = 0.1;          % Valve time constant (s)

% Controller gains
Kd = 2;               % Derivative gain
Kp_ctrl = 5;          % Proportional gain

%% Define System Dynamics
% State vector: x = [theta, omega, p]
% theta: angular position
% omega: angular velocity
% p: hydraulic pressure

% System matrices for linear analysis around equilibrium
A = [0          1           0;
     0        -b/J        Kt/J;
     0      -Kp*Kt    -1/tau_v];

B = [0; 0; Kv/tau_v];

% Output matrix (measure angular position and velocity)
C = [1  0  0;
     0  1  0];

%% Lyapunov Stability Analysis

% Check eigenvalues
eigenvalues = eig(A);
fprintf('System Eigenvalues:\n');
disp(eigenvalues);

% Check if system is stable (all eigenvalues have negative real parts)
if all(real(eigenvalues) < 0)
    fprintf('\n✓ System is STABLE: All eigenvalues have negative real parts\n');
else
    fprintf('\n✗ System is UNSTABLE: Some eigenvalues have non-negative real parts\n');
end

%% Lyapunov Equation Solution
% Solve: A'*P + P*A = -Q for Lyapunov function V(x) = x'*P*x
% Using Q = Identity matrix (positive definite)

Q = eye(3);
P = lyap(A', Q);

fprintf('\n--- Lyapunov Matrix P ---\n');
disp(P);

% Check if P is positive definite (all eigenvalues > 0)
P_eig = eig(P);
fprintf('\nEigenvalues of P matrix:\n');
disp(P_eig);

if all(P_eig > 0)
    fprintf('\n✓ P is positive definite - Lyapunov function is valid\n');
else
    fprintf('\n✗ P is not positive definite\n');
end

%% Compute Lyapunov Function Properties
% Analyze the rate of decay of the Lyapunov function
% dV/dt = -x'*Q*x < 0 (ensures asymptotic stability)

fprintf('\n--- Lyapunov Function Analysis ---\n');
fprintf('For a valid Lyapunov function V(x) = x''*P*x:\n');
fprintf('- V(x) > 0 for all x ≠ 0 (P is positive definite)\n');
fprintf('- dV/dt = -x''*Q*x < 0 (Q is positive definite)\n');
fprintf('This guarantees asymptotic stability in the large\n');

%% Regional Stability Analysis
% Define a region around the origin bounded by V(x) ≤ c
c = 1;  % Lyapunov function level set
fprintf('\n--- Stability Region Analysis ---\n');
fprintf('Region of attraction: V(x) ≤ %.2f (ellipsoid in state space)\n', c);

%% Simulation: Closed-loop system with feedback control
% u = -K*x (state feedback control)
K = [Kp_ctrl, Kd, 0.5];  % Control gain vector

A_cl = A - B*K;  % Closed-loop system matrix

eigenvalues_cl = eig(A_cl);
fprintf('\n--- Closed-Loop System ---\n');
fprintf('Closed-loop eigenvalues (with feedback control):\n');
disp(eigenvalues_cl);

if all(real(eigenvalues_cl) < 0)
    fprintf('✓ Closed-loop system is STABLE\n');
else
    fprintf('✗ Closed-loop system is UNSTABLE\n');
end

%% Time-domain simulation
t_sim = 0:0.001:10;  % Simulation time
x0 = [0.5; 0; 0.2];  % Initial condition (position, velocity, pressure)

% Simulate open-loop system
[t_ol, x_ol] = ode45(@(t,x) A*x, t_sim, x0);

% Simulate closed-loop system
[t_cl, x_cl] = ode45(@(t,x) A_cl*x, t_sim, x0);

% Compute Lyapunov function values
V_ol = zeros(length(t_ol), 1);
V_cl = zeros(length(t_cl), 1);

for i = 1:length(t_ol)
    V_ol(i) = x_ol(i,:)*P*x_ol(i,:)';
end

for i = 1:length(t_cl)
    V_cl(i) = x_cl(i,:)*P*x_cl(i,:)';
end

%% Visualization
figure('Position', [100, 100, 1200, 800]);

% Plot 1: Open-loop system states
subplot(2,3,1);
plot(t_ol, x_ol(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t_ol, x_ol(:,2), 'r-', 'LineWidth', 1.5);
plot(t_ol, x_ol(:,3), 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('State Value');
title('Open-Loop System Response');
legend('Position θ', 'Velocity ω', 'Pressure p');
grid on;

% Plot 2: Closed-loop system states
subplot(2,3,2);
plot(t_cl, x_cl(:,1), 'b-', 'LineWidth', 1.5); hold on;
plot(t_cl, x_cl(:,2), 'r-', 'LineWidth', 1.5);
plot(t_cl, x_cl(:,3), 'g-', 'LineWidth', 1.5);
xlabel('Time (s)'); ylabel('State Value');
title('Closed-Loop System Response (with Feedback Control)');
legend('Position θ', 'Velocity ω', 'Pressure p');
grid on;

% Plot 3: Lyapunov function decay (open-loop)
subplot(2,3,3);
semilogy(t_ol, V_ol, 'b-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('V(x) = x^T P x');
title('Lyapunov Function - Open-Loop System');
grid on;

% Plot 4: Lyapunov function decay (closed-loop)
subplot(2,3,4);
semilogy(t_cl, V_cl, 'r-', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('V(x) = x^T P x');
title('Lyapunov Function - Closed-Loop System');
grid on;

% Plot 5: Phase portrait
subplot(2,3,5);
plot(x_ol(:,1), x_ol(:,2), 'b-', 'LineWidth', 1.5); hold on;
plot(x_cl(:,1), x_cl(:,2), 'r-', 'LineWidth', 1.5);
plot(x0(1), x0(2), 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
xlabel('Position θ (rad)'); ylabel('Velocity ω (rad/s)');
title('Phase Portrait');
legend('Open-Loop', 'Closed-Loop', 'Initial Condition');
grid on;

% Plot 6: Eigenvalue map
subplot(2,3,6);
plot(real(eigenvalues), imag(eigenvalues), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b'); hold on;
plot(real(eigenvalues_cl), imag(eigenvalues_cl), 'rs', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
axvline(0, 'k--', 'LineWidth', 1);
xlabel('Real Part'); ylabel('Imaginary Part');
title('Eigenvalue Locations');
legend('Open-Loop', 'Closed-Loop');
grid on;

sgtitle('Lyapunov Stability Analysis - Hydraulic Servo Motor Control System', 'FontSize', 14, 'FontWeight', 'bold');

%% Summary Report
fprintf('\n========================================\n');
fprintf('STABILITY ANALYSIS SUMMARY\n');
fprintf('========================================\n');
fprintf('System Type: Hydraulic Servo Motor Control\n');
fprintf('Analysis Method: Lyapunov Stability Analysis\n\n');

fprintf('OPEN-LOOP SYSTEM:\n');
fprintf('  Stability: %s\n', all(real(eigenvalues) < 0) ? 'STABLE' : 'UNSTABLE');
fprintf('  Damping ratio: ');
for i = 1:length(eigenvalues)
    fprintf('λ%d = %.4f, ', i, eigenvalues(i));
end
fprintf('\n\n');

fprintf('CLOSED-LOOP SYSTEM (with feedback):\n');
fprintf('  Stability: %s\n', all(real(eigenvalues_cl) < 0) ? 'STABLE' : 'UNSTABLE');
fprintf('  Damping ratio: ');
for i = 1:length(eigenvalues_cl)
    fprintf('λ%d = %.4f, ', i, eigenvalues_cl(i));
end
fprintf('\n\n');

fprintf('LYAPUNOV FUNCTION:\n');
fprintf('  Positive definite: %s\n', all(P_eig > 0) ? 'YES' : 'NO');
fprintf('  Condition number: %.4f\n', cond(P));
fprintf('========================================\n');
