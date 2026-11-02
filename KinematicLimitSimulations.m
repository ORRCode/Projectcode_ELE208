
clc;
clear;
close all;

%% =========================================
%  1. CREATE ROBOT MODEL
% ==========================================

% Standard DH parameters
% Units: meters and radians

J(1) = Link('d', 0.2, 'a',  0,   'alpha', pi/2);
J(2) = Link('d', 0,   'a', -0.6, 'alpha', 0);
J(3) = Link('d', 0,   'a', -0.6, 'alpha', 0);
J(4) = Link('d', 0.2, 'a',  0,   'alpha', pi/2);
J(5) = Link('d', 0.1, 'a',  0,   'alpha', -pi/2);
J(6) = Link('d', 0.1, 'a',  0,   'alpha', 0);

% Create robot
robot = SerialLink(J, 'name', 'Industrial Robot');

%% =========================================
%  2. DEFINE CARTESIAN POSITIONS
% ==========================================

%homeTCP = [0.45 0.60 0.45];
%ready   = [0.45 1.00 0.57];
%Pick up = [0.00 1,00 0.02];
%retract = [0.30 0.65 0.70];
%middle = [0.60 0 0.7];
%deliver = [0.00 1.20 0.20];

% START POSITION (meters)
start_x = 0.3;
start_y = 0.65;
start_z = 0.7;

% TARGET POSITION (meters)
target_x = 0.6;
target_y = 0;
target_z = 0.7;

% Create transformation matrices
T_start = transl(start_x, start_y, start_z);
T_target = transl(target_x, target_y, target_z);


%% =========================================
%  3. INVERSE KINEMATICS
% ==========================================

% Initial guess
q_guess = zeros(1,6);

% Calculate starting configuration
q_start = robot.ikine(T_start, ...
    'q0', q_guess, ...
    'mask', [1 1 1 0 0 0]);

% Calculate target configuration
q_target = robot.ikine(T_target, ...
    'q0', q_start, ...
    'mask', [1 1 1 0 0 0]);

% Check if solutions were found
if isempty(q_start) || isempty(q_target)
    error('Inverse kinematics did not find a solution.');
end


%% =========================================
%  4. FORWARD KINEMATICS VERIFICATION
% ==========================================

T_actual_start = robot.fkine(q_start);
T_actual_target = robot.fkine(q_target);

actual_start = transl(T_actual_start);
actual_target = transl(T_actual_target);

% Calculate position errors
position_error_start = norm( ...
    [start_x start_y start_z] - actual_start);

position_error_target = norm( ...
    [target_x target_y target_z] - actual_target);

disp('========================================');
disp('         KINEMATICS RESULTS');
disp('========================================');

disp('Desired start position:');
disp([start_x start_y start_z]);

disp('Actual start position:');
disp(actual_start);

disp('Desired target position:');
disp([target_x target_y target_z]);

disp('Actual target position:');
disp(actual_target);

disp('Start position error (m):');
disp(position_error_start);

disp('Target position error (m):');
disp(position_error_target);

disp('Start joint angles (degrees):');
disp(rad2deg(q_start));

disp('Target joint angles (degrees):');
disp(rad2deg(q_target));

%% =========================================
%  5. DISPLAY START POSITION
% ==========================================

figure;

robot.plot(q_start);

title('Robot - Start Position');

%% =========================================
%  6. DISPLAY TARGET POSITION
% ==========================================

figure;

robot.plot(q_target);

title('Robot - Target Position');

%% =========================================
%  7. MOTION PLANNING
% ==========================================

% Number of trajectory points
steps = 100;

% Generate smooth joint trajectory
trajectory = jtraj(q_start, q_target, steps);

%% =========================================
%  8. ANIMATE ROBOT MOVEMENT
% ==========================================

figure;

robot.plot(trajectory);

title('Pick and Place - Robot Motion');

%% =========================================
%  9. SIMULATION SUMMARY
% ==========================================

disp('========================================');
disp('          SIMULATION COMPLETED');
disp('========================================');

disp('Start position:');
disp([start_x start_y start_z]);

disp('Target position:');
disp([target_x target_y target_z]);

disp('Number of trajectory points:');
disp(steps);

disp('Simulation finished successfully.');
