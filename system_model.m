%% Hydraulic Servo Motor Control System Model
% This script defines the system parameters and state space representation
% for a hydraulic servo motor control system
%
% State variables:
%   x1: Motor angular position (rad)
%   x2: Motor angular velocity (rad/s)
%   x3: Valve spool position (m)
%   x4: Hydraulic pressure (Pa)
%
% Input:
%   u: Control voltage to valve actuator (V)
%
% Output:
%   y: Motor angular position (rad)

clear all; close all; clc;

%% System Parameters for Hydraulic Servo Motor Control

% Motor Parameters
J_m = 0.5;              % Motor inertia (kg*m^2)
b_m = 0.2;              % Viscous friction coefficient (N*m*s/rad)
K_m = 0.8;              % Motor torque coefficient (N*m/A)

% Hydraulic Valve Parameters
K_v = 5e-5;             % Valve flow gain (m^3/(s*A))
m_v = 0.05;             % Valve spool mass (kg)
b_v = 0.8;              % Valve damping coefficient (N*s/m)
K_sv = 100;             % Solenoid force coefficient (N/A)

% Hydraulic System Parameters
C_t = 1e-11;            % Total system compliance (m^5/N)
rho = 860;              % Hydraulic fluid density (kg/m^3)
D_m = 50e-6;            % Motor displacement (m^3/rev)
P_s = 20e6;             % Supply pressure (Pa)
P_r = 0;                % Return pressure (Pa)

% Actuator/Sensor Parameters
K_a = 10;               % Amplifier gain (A/V)
R_v = 5;                % Valve resistance (Ohm)
L_v = 0.05;             % Valve inductance (H)

% Load and Environmental Parameters
T_L = 0;                % External load torque (N*m)
theta_0 = 0;            % Initial position (rad)
omega_0 = 0;            % Initial velocity (rad/s)

%% State Space Representation
% General form: dx/dt = A*x + B*u
%               y = C*x + D*u

% State vector: x = [x1; x2; x3; x4]
%   x1: Motor angular position (rad)
%   x2: Motor angular velocity (rad/s)
%   x3: Valve spool position (m)
%   x4: Hydraulic pressure (Pa)

% System matrices
A = [0,                  1,              0,              0;
     0,                  -b_m/J_m,       K_m*D_m/J_m,    0;
     0,                  0,              -b_v/m_v,       -K_v/(m_v);
     K_v*P_s/C_t,        0,              -K_v*D_m/C_t,   0];

B = [0;
     0;
     K_sv*K_a/m_v;
     0];

C = [1, 0, 0, 0];       % Output: motor position

D = 0;                  % No direct feedthrough

%% Display System Information
fprintf('\n===== Hydraulic Servo Motor Control System Model =====\n\n');

fprintf('System Dimensions:\n');
fprintf('  Number of states: %d\n', length(A));
fprintf('  Number of inputs: %d\n', size(B, 2));
fprintf('  Number of outputs: %d\n', size(C, 1));

fprintf('\nMotor Parameters:\n');
fprintf('  Inertia (J_m): %.4f kg*m^2\n', J_m);
fprintf('  Friction coefficient (b_m): %.4f N*m*s/rad\n', b_m);
fprintf('  Torque coefficient (K_m): %.4f N*m/A\n', K_m);

fprintf('\nValve Parameters:\n');
fprintf('  Flow gain (K_v): %.2e m^3/(s*A)\n', K_v);
fprintf('  Spool mass (m_v): %.4f kg\n', m_v);
fprintf('  Damping (b_v): %.4f N*s/m\n', b_v);
fprintf('  Solenoid gain (K_sv): %.2f N/A\n', K_sv);

fprintf('\nHydraulic Parameters:\n');
fprintf('  System compliance (C_t): %.2e m^5/N\n', C_t);
fprintf('  Fluid density (rho): %.2f kg/m^3\n', rho);
fprintf('  Motor displacement (D_m): %.2e m^3/rev\n', D_m);
fprintf('  Supply pressure (P_s): %.2e Pa\n', P_s);

fprintf('\nState Space Matrices:\n');
fprintf('A matrix (dynamics):\n');
disp(A);

fprintf('B matrix (input influence):\n');
disp(B);

fprintf('C matrix (output selection):\n');
disp(C);

fprintf('D matrix (feedthrough):\n');
disp(D);

%% Eigenvalue Analysis
eigenvalues = eig(A);
fprintf('\nSystem Eigenvalues:\n');
for i = 1:length(eigenvalues)
    fprintf('  λ%d = %.4f\n', i, real(eigenvalues(i)));
end

% Check stability
if all(real(eigenvalues) < 0)
    fprintf('\nSystem Status: STABLE (all eigenvalues have negative real parts)\n');
else
    fprintf('\nSystem Status: UNSTABLE (some eigenvalues have positive real parts)\n');
end

%% Create state space model
sys = ss(A, B, C, D);
fprintf('\nState Space Model Created:\n');
disp(sys);

%% Save system parameters to file
save('system_params.mat', 'A', 'B', 'C', 'D', 'J_m', 'b_m', 'K_m', ...
    'K_v', 'm_v', 'b_v', 'K_sv', 'C_t', 'rho', 'D_m', 'P_s', 'P_r', ...
    'K_a', 'R_v', 'L_v', 'T_L');

fprintf('\nSystem parameters saved to system_params.mat\n');
