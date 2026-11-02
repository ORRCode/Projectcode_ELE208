
clc;
clear;
close all;

%% 1. Define robot using DH parameters

J(1) = Link('d', 0.2, 'a',  0,   'alpha', pi/2);
J(2) = Link('d', 0,   'a', -0.6, 'alpha', 0);
J(3) = Link('d', 0,   'a', -0.6, 'alpha', 0);
J(4) = Link('d', 0.2, 'a',  0,   'alpha', pi/2);
J(5) = Link('d', 0.1, 'a',  0,   'alpha', -pi/2);
J(6) = Link('d', 0.1, 'a',  0,   'alpha', 0);

robot = SerialLink(J, 'name', 'Industrial Robot');

%% 2. Define required TCP positions (x, y, z)

P = [
    0.45  0.60  0.45;  % Home TCP
    0.45  1.00  0.57;  % Ready
    0.00  1.00  0.02;  % Pick up
    0.30  0.65  0.70;  % Retract
    -0.60  0.00  0.70;  % Middle
    0.00  -1.20  0.20   % Deliver
];

pointNames = {
    'Home TCP'
    'Ready'
    'Pick up'
    'Retract'
    'Middle'
    'Deliver'
};

%% 3. Calculate inverse kinematics for each position

% Only constrain X, Y and Z.
% Orientation is not specified in the assignment's coordinates.
mask = [1 1 1 0 0 0];

q_waypoints = zeros(6,6);

% Initial joint configuration
q0 = zeros(1,6);

for i = 1:6

    % Create target transformation
    T = transl(P(i,1), P(i,2), P(i,3));

    % Calculate inverse kinematics
    q = robot.ikine(T, 'q0', q0, 'mask', mask);

    % Check that a solution was found
    if any(~isfinite(q))
        error('IK failed at position: %s', pointNames{i});
    end

    % Check actual position using forward kinematics
    T_actual = robot.fkine(q);
    p_actual = transl(T_actual);
    p_actual = p_actual(:).';

    positionError = norm(p_actual - P(i,:));

    if positionError > 1e-3
        error('Position error too large at: %s', pointNames{i});
    end

    % Save joint angles
    q_waypoints(i,:) = q;

    % Use previous solution as next initial guess
    q0 = q;

    fprintf('%s: IK solution found. Error = %.6f m\n', ...
        pointNames{i}, positionError);
end

%% 4. Motion planning between positions

% Number of trajectory steps between each pair of points
N = 50;

Q = [];

for i = 1:5

    % Generate smooth joint trajectory
    q_trajectory = jtraj( ...
        q_waypoints(i,:), ...
        q_waypoints(i+1,:), ...
        N);

    % Avoid duplicating the connecting point
    if i > 1
        q_trajectory = q_trajectory(2:end,:);
    end

    % Add trajectory to complete motion
    Q = [Q; q_trajectory];

end

%% 5. Animate the robot

figure;

robot.plot(Q, ...
    'workspace', [-1.5 1.5 -1.5 1.5 -0.2 1.5], ...
    'delay', 0.02);

title('Robot Motion Simulation');

%% 6. Plot end-effector path

tcpPath = zeros(size(Q,1),3);

for i = 1:size(Q,1)

    T = robot.fkine(Q(i,:));
    p = transl(T);

    tcpPath(i,:) = p(:).';

end

figure;

plot3(tcpPath(:,1), tcpPath(:,2), tcpPath(:,3), ...
    'LineWidth', 2);

hold on;

plot3(P(:,1), P(:,2), P(:,3), ...
    'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

grid on;
axis equal;

xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');

title('End-Effector Trajectory');

legend('TCP trajectory', 'Required positions');
