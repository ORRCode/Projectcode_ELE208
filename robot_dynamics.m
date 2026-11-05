
%% ROBOT DYNAMICS AND CONTROL
% Peter Corke Robotics Toolbox
% Includes payload, PID control and RNE inverse dynamics

clear;
clc;
close all;

%% 1. DEFINE DH MODEL

J(1) = Link('d', 0.2, 'a',  0,    'alpha', pi/2);
J(2) = Link('d', 0,   'a', -0.6,  'alpha', 0);
J(3) = Link('d', 0,   'a', -0.6,  'alpha', 0);
J(4) = Link('d', 0.2, 'a',  0,    'alpha', pi/2);
J(5) = Link('d', 0.1, 'a',  0,    'alpha', -pi/2);
J(6) = Link('d', 0.1, 'a',  0,    'alpha', 0);

robot = SerialLink(J, 'name', 'Industrial Robot');

%% 2. ESTIMATED DYNAMIC PARAMETERS

% Estimated link masses [kg]
mass = [5 8 5 2.5 1.5 1];

% Centre of mass [m]
COM = [
     0       0    0.10;
    -0.30    0    0;
    -0.30    0    0;
     0       0    0.10;
     0       0    0.05;
     0       0    0.05
];

% Estimated inertia components [kg*m^2]
I = [
    0.020  0.020  0.010;
    0.010  0.250  0.250;
    0.005  0.150  0.150;
    0.005  0.005  0.003;
    0.001  0.001  0.001;
    0.001  0.001  0.001
];

% Motor inertia [kg*m^2]
Jm = [2e-5 2e-5 1.5e-5 8e-6 5e-6 3e-6];

% Gear ratios
G = [100 100 80 60 50 40];

% Viscous friction
B = [0.1 0.1 0.08 0.05 0.03 0.02];

% Coulomb friction
Tc = [0.2 0.2 0.15 0.1 0.05 0.03];

% Assign parameters to robot links
for i = 1:6
    J(i).m = mass(i);
    J(i).r = COM(i,:);
    J(i).I = diag(I(i,:));
    J(i).Jm = Jm(i);
    J(i).G = G(i);
    J(i).B = B(i);
    J(i).Tc = [Tc(i) -Tc(i)];
end

% Gravity
robot.gravity = [0 0 9.81];

disp('Robot dynamic model created.');

%% 3. PAYLOAD

% Gripper mass = 2 kg
% Object mass = 5 kg
% Total payload after pickup = 7 kg

gripper_mass = 2;
object_mass = 5;
total_payload = gripper_mass + object_mass;

% Estimated payload centre of mass in tool coordinates [m]
payload_COM = [0 0 0.1];

% Use maximum payload for the transfer function model
robot.payload(total_payload, payload_COM);

disp('Maximum payload:');
disp(total_payload);

%% 4. DEFINE MOVEMENT POINTS

P = [
    0.45  0.60  0.45;  % Home TCP
    0.45  1.00  0.57;  % Ready
    0.00  1.00  0.02;  % Pick up
    0.30  0.65  0.70;  % Retract
   -0.60  0.00  0.70;  % Middle
    0.00 -1.20  0.20   % Deliver
];

mask = [1 1 1 0 0 0];

q_waypoints = zeros(6,6);
q0 = zeros(1,6);

%% 5. INVERSE KINEMATICS

for k = 1:size(P,1)

    T = transl(P(k,1), P(k,2), P(k,3));

    q = robot.ikine(T, 'q0', q0, 'mask', mask);

    if isempty(q) || any(~isfinite(q))
        error('Inverse kinematics failed at waypoint %d.', k);
    end

    % Keep joint angles close to previous solution
    q = q0 + atan2(sin(q-q0), cos(q-q0));

    q_waypoints(k,:) = q;
    q0 = q;

end

disp('Joint waypoint solutions [rad]:');
disp(q_waypoints);

%% 6. CHECK TCP POSITION ERRORS

position_error = zeros(6,1);

for k = 1:6

    T_actual = robot.fkine(q_waypoints(k,:));
    actual_position = transl(T_actual);

    position_error(k) = norm(P(k,:) - actual_position);

end

position_error_mm = position_error * 1000;

disp('TCP position errors [mm]:');
disp(position_error_mm);

% Check pick-up position error
pickup_error = position_error_mm(3);

fprintf('Pick-up position error: %.3f mm\n', pickup_error);

if pickup_error < 10
    disp('Pick-up position requirement: PASSED');
else
    disp('Pick-up position requirement: FAILED');
end

% Check all waypoint position errors
if max(position_error_mm) < 5
    disp('TCP position requirement (<5 mm): PASSED');
else
    disp('TCP position requirement (<5 mm): FAILED');
end

%% 7. CREATE COMPLETE JOINT TRAJECTORY

N = 50;
segment_time = 2;

Q = [];
QD = [];
QDD = [];
t_all = [];
tau = [];

for k = 1:size(q_waypoints,1)-1

    t_segment = linspace(0, segment_time, N);

    [Qseg, QDseg, QDDseg] = jtraj( ...
        q_waypoints(k,:), ...
        q_waypoints(k+1,:), ...
        t_segment);

    % Remove duplicated samples
    if k > 1
        Qseg = Qseg(2:end,:);
        QDseg = QDseg(2:end,:);
        QDDseg = QDDseg(2:end,:);
        t_segment = t_segment(2:end);
    end

    % Payload before and after pickup
    % Segments 1 and 2: robot carries gripper only
    % Segments 3, 4 and 5: robot carries gripper and object

    if k < 3
        current_payload = gripper_mass;
    else
        current_payload = total_payload;
    end

    % Update payload in dynamic model
    robot.payload(current_payload, payload_COM);

    % Recursive Newton-Euler calculation for this segment
    tau_segment = robot.rne(Qseg, QDseg, QDDseg);

    % Save trajectory and torque data
    Q = [Q; Qseg];
    QD = [QD; QDseg];
    QDD = [QDD; QDDseg];
    tau = [tau; tau_segment];

    t_all = [t_all; t_segment(:) + (k-1)*segment_time];

end

disp('Joint trajectory created.');

%% 8. ANALYSE JOINT DYNAMICS

% Use maximum payload for worst-case analysis
robot.payload(total_payload, payload_COM);

q_test = q_waypoints(1,:);
qd_test = [0.2 -0.3 0.2 0.1 -0.1 0.15];

% Inertia matrix
M = robot.inertia(q_test);

% Coriolis matrix
Cmat = robot.coriolis(q_test, qd_test);

% Gravity torque
gravity_torque = robot.gravload(q_test);

disp('Inertia matrix M:');
disp(M);

disp('Coriolis matrix C:');
disp(Cmat);

disp('Gravity torque [Nm]:');
disp(gravity_torque);

%% 9. JOINT 2 TRANSFER FUNCTION

% Simplified independent joint model
J_eq = M(2,2);
B_eq = B(2);

s = tf('s');

G_joint = 1/(J_eq*s^2 + B_eq*s);

disp('Joint 2 transfer function:');
G_joint

figure;
step(G_joint);
grid on;
title('Open-loop step response - Joint 2');
xlabel('Time [s]');
ylabel('Joint angle [rad]');

%% 10. PID POSITION CONTROLLER FOR JOINT 2

% Desired closed-loop dynamics
zeta = 3.5;
wn = 8;
alpha = 10;

% PID gains using desired characteristic polynomial
Kd = J_eq*(alpha + 2*zeta*wn) - B_eq;
Kp = J_eq*(wn^2 + 2*zeta*wn*alpha);
Ki = J_eq*alpha*wn^2;

controller = pid(Kp, Ki, Kd);

% Closed-loop system
closed_loop = feedback(controller*G_joint, 1);

disp('PID controller gains:');
fprintf('Kp = %.4f\n', Kp);
fprintf('Ki = %.4f\n', Ki);
fprintf('Kd = %.4f\n', Kd);

disp('Closed-loop transfer function:');
closed_loop

figure;
step(closed_loop);
grid on;
title('Closed-loop PID response - Joint 2');
xlabel('Time [s]');
ylabel('Joint angle [rad]');

%% 11. CHECK POSITION CONTROL REQUIREMENTS

response_info = stepinfo(closed_loop);

% Steady-state error for a unit step
steady_state_error = abs(1 - dcgain(closed_loop));

% Convert radians to degrees
steady_state_error_deg = steady_state_error * 180/pi;

disp('PID response characteristics:');
disp(response_info);

fprintf('Steady-state error: %.5f degrees\n', ...
    steady_state_error_deg);

fprintf('Settling time: %.4f seconds\n', ...
    response_info.SettlingTime);

fprintf('Overshoot: %.4f %%\n', ...
    response_info.Overshoot);

% Check all requirements
if steady_state_error_deg < 1
    disp('Rotational steady-state error: PASSED');
else
    disp('Rotational steady-state error: FAILED');
end

if response_info.SettlingTime < 1
    disp('Settling time: PASSED');
else
    disp('Settling time: FAILED');
end

if response_info.Overshoot < 10
    disp('Overshoot: PASSED');
else
    disp('Overshoot: FAILED');
end

%% 12. RECURSIVE NEWTON-EULER RESULTS

disp('Recursive Newton-Euler calculation completed.');

%% 13. PLOT REQUIRED ACTUATOR TORQUES

figure;

plot(t_all, tau, 'LineWidth', 1.2);
grid on;

title('Required actuator torques - Recursive Newton-Euler');
xlabel('Time [s]');
ylabel('Torque [Nm]');

legend('Joint 1','Joint 2','Joint 3', ...
       'Joint 4','Joint 5','Joint 6', ...
       'Location','best');

%% 14. MAXIMUM AND RMS TORQUE

peak_torque = max(abs(tau), [], 1);
rms_torque = sqrt(mean(tau.^2, 1));

disp('Maximum absolute torque [Nm]:');
disp(peak_torque);

disp('RMS torque [Nm]:');
disp(rms_torque);

%% 15. PLOT JOINT POSITIONS

figure;

plot(t_all, Q, 'LineWidth', 1.2);
grid on;

title('Joint positions during robot movement');
xlabel('Time [s]');
ylabel('Joint angle [rad]');

legend('Joint 1','Joint 2','Joint 3', ...
       'Joint 4','Joint 5','Joint 6', ...
       'Location','best');

%% 16. ANIMATE ROBOT

figure;
robot.plot(Q);

title('Robot movement with payload');

disp('Robot dynamics and control simulation completed.');
