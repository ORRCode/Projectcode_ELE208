% Run "startup_rvc.n" in Peter Corke's toolbox to use this script.

% KI er benytta som hjelpemiddel for å utvikle dette programmet. Gruppa har
% i hovedsak benytta det til feilsøking, optimalisering av funksjoner og
% og for å forstå funksjoner som ble anbefalt.

%% ===================== VARIABLAR =====================
clear; clc; close all;

L(1) = Link('d', 0.2, 'a',  0,   'alpha',  pi/2);
L(2) = Link('d', 0,   'a', -0.6, 'alpha',  0);
L(3) = Link('d', 0,   'a', -0.6, 'alpha',  0);
L(4) = Link('d', 0.2, 'a',  0,   'alpha',  pi/2);
L(5) = Link('d', 0.1, 'a',  0,   'alpha', -pi/2);
L(6) = Link('d', 0.1, 'a',  0,   'alpha',  0);
robot = SerialLink(L, 'name', 'ELE208 robotarm');
robot.tool = transl(0, 0, 0.196); 
robot.qlim = [-2*pi 2*pi; repmat([-pi pi], 5, 1)];   

cfg.zBase   = 0.5;                       
cfg.zBoxTop = 0.454 - cfg.zBase;        
cfg.xPick   = 0.3;   cfg.yBelt = 1.0;    
cfg.hHover  = 0.12;                      
cfg.pallet  = [0 -1.0];                  
cfg.zPlace  = 0.344 - cfg.zBase + 0.005; 
cfg.hAbove  = 0.15;                      

vBelt = -0.3;                     
par.tc    = -0.7;                 
par.Tc    = 1.0;                  
par.Td    = 0.30;                 
par.dwell = 0.30;                 
par.Tl    = 0.40;                 
par.hLift = 0.15;                 
par.Tdec  = 0.50;                 
par.dt    = 0.02;                 
par.tLag = 0.037;                 
par.xWait = cfg.xPick + abs(vBelt)*(-par.tc) + abs(vBelt)*par.Tc/2;   

P.wait  = [par.xWait  cfg.yBelt  cfg.zBoxTop + cfg.hHover];
P.pick  = [cfg.xPick  cfg.yBelt  cfg.zBoxTop];
P.above = [cfg.pallet cfg.zPlace + cfg.hAbove];
P.place = [cfg.pallet cfg.zPlace];

pose = @(p) transl(p) * trotx(pi);      

names = {'wait','pick','above','place'};

seed.wait  = deg2rad([-118 -49  77 -118 -90  -28]);
seed.above = deg2rad([  78 -49  85 -126 -90  168]);

Q = struct();
qPrev = seed.wait;

jn = ["foundation_to_base","shoulder","elbow","wrist1","wrist2","wrist3"];

vmax = [2.09 2.09 3.14 3.14 3.14 3.14];         
seq  = {'pick','wait','above','place','above','wait'};


cam.pos  = [0.8 1.0 1.8];                 
cam.fov  = 1.57;   cam.W = 320;  cam.H = 240;
cam.box  = 0.2;                           
cam.zTop = 0.454;                         
cam.s    = 2*(cam.pos(3)-cam.zTop)*tan(cam.fov/2)/cam.W;   


xDet  = 1.55;                
vBelt = -0.3;                 
vTol = 0.03;                  

vPlace = 0.15;                    

%% ===================== PROGRAM =====================

for i = 1:numel(names)
    name = names{i};
    T = pose(P.(name));

    switch name
        case 'wait',  seed_i = seed.wait;
        case 'above', seed_i = seed.above;
        otherwise,    seed_i = qPrev;      
    end

    q = robot.ikine(T,'q0',seed_i,'ilimit',1000,'rlimit',200);
    q = atan2(sin(q),cos(q));

    Tchk = double(robot.fkine(q));
    err  = norm(Tchk(1:3,4) - T(1:3,4)) * 1000;      
    Tel    = robot.A(1:2,q);
    zElbow = Tel.t(3) + cfg.zBase;                   
    assert(zElbow > 1.0, '%s: albogen er ikkje opp (z = %.2f m)', name, zElbow);

    if any(strcmp(name,{'above','place'}))
        assert(q(1) > 0, '%s: feil grein, q1 skal vere > 0', name);
    end

    Q.(name) = q;
    qPrev = q;
    fprintf('%-6s q [deg] = %s   feil = %.3f mm   albogledd z = %.2f m\n', ...
            name, mat2str(round(rad2deg(q))), err, zElbow);
end

% Om ikkje den køyrer får før, start Gazebo
startsekvens = input('Is the sim running? (y/n)', 's');
if ~strcmpi(strtrim(startsekvens), 'y')
    disp('Canceled.');
    return
end

% Start nodes and send arm to waiting pos
node = ros2node("/matlab_ctrl", 8);
detachPub = ros2publisher(node, "/vacuum/detach", "std_msgs/Empty");
attachPub = ros2publisher(node, "/vacuum/attach", "std_msgs/Empty");

pub  = ros2publisher(node, "/joint_trajectory_controller/joint_trajectory", ...
                     "trajectory_msgs/JointTrajectory");
sub  = ros2subscriber(node, "/joint_states", "sensor_msgs/JointState");

beltPub = ros2publisher(node, "/belt/cmd_vel", "geometry_msgs/Twist");
imgSub  = ros2subscriber(node, "/conveyor_camera/image", "sensor_msgs/Image");

pause(2);  % Delay for the nodes to connect
send(detachPub, ros2message(detachPub));    
pause(2); 

readQ = @() local_readQ(sub, jn);

msg = ros2message(pub);
msg.joint_names = cellstr(jn);
pt  = ros2message("trajectory_msgs/JointTrajectoryPoint");
pt.positions     = Q.wait;
pt.velocities    = zeros(1,0);
pt.accelerations = zeros(1,0);
pt.effort        = zeros(1,0);
pt.time_from_start.sec     = int32(6);
pt.time_from_start.nanosec = uint32(0);
msg.points = pt;
send(pub, msg);

msg.points(1).positions = Q.wait;
send(pub, msg);
ok = waitForPose(readQ, Q.wait, 0.1, 60);
fprintf('ved wait: %d\n', ok);

qSim = readQ();
Tsim = double(robot.fkine(qSim));
Tref = double(robot.fkine(Q.wait));
fprintf('q simulert [deg] = %s\n', mat2str(round(rad2deg(qSim),1)));
fprintf('q ønskt    [deg] = %s\n', mat2str(round(rad2deg(Q.wait),1)));
fprintf('maks leddfeil    = %.2f deg\n', max(abs(rad2deg(qSim-Q.wait))));
fprintf('TCP fkine(qSim)  = %s m\n', mat2str(round(Tsim(1:3,4)',3)));
fprintf('TCP ønskt        = %s m\n', mat2str(round(Tref(1:3,4)',3)));

% Sekvens oppgåve

startPickupPlace = input('Start sequence. (y/n)','s');
if ~strcmpi(strtrim(startPickupPlace), 'y')
    disp('Canceled.');
    return
end

plan = planPick(robot, cfg, par, vBelt, Q.wait);

err = zeros(size(plan.tau));
for i = 1:numel(plan.tau)
    Ti = double(robot.fkine(plan.q(i,:)));
    err(i) = norm(Ti(1:3,4) - [plan.X(i); cfg.yBelt; plan.Z(i)]);
end
vx = gradient(plan.X, par.dt);   vz = gradient(plan.Z, par.dt);
sp = hypot(vx, vz);
dn = plan.tau >= -par.Td & plan.tau <= 0;

fprintf('punkt: %d   varigheit: %.2f s\n', numel(plan.tau), plan.tau(end) - plan.tau(1));
fprintf('maks IK-feil:                %.4f mm\n', 1000*max(err));
fprintf('maks leddfart / grense:      %s\n', mat2str(round(max(abs(plan.qd)) ./ vmax, 2)));
fprintf('TCP-fart i verda under ned:  snitt %.2f   min %.2f   maks %.2f m/s\n', mean(sp(dn)), min(sp(dn)), max(sp(dn)));
i0 = find(plan.tau >= 0, 1);
fprintf('q ved start [deg]: %s\n', mat2str(round(rad2deg(plan.q(1,:)))));
fprintf('q ved tau=0 [deg]: %s\n', mat2str(round(rad2deg(plan.q(i0,:)))));

% figure; subplot(2,1,1); plot(plan.tau, plan.q*180/pi); grid on; ylabel('q [deg]'); legend('q1','q2','q3','q4','q5','q6');
% subplot(2,1,2); plot(plan.tau, [plan.X plan.Z]); grid on; xlabel('tid relativt til t_{pick} [s]'); ylabel('TCP [m]'); legend('x','z');

mb = ros2message(beltPub);  mb.linear.x = vBelt;
send(beltPub, mb);                                   

buf = zeros(0,2);  found = false;  tic;
while toc < 40
    m = receive(imgSub, 10);
    t = stampSec(m);
    [xm, ~, ok] = detectBox(rosReadImage(m), cam);
    if ~ok, buf = zeros(0,2); continue; end
    buf = [buf; t xm];
    if size(buf,1) > 3, buf(1,:) = []; end
    if size(buf,1) == 3 && xm <= xDet && xm > 1.0
        pf = polyfit(buf(:,1) - buf(1,1), buf(:,2), 1);
        if abs(pf(1) - vBelt) < vTol, found = true; break; end
    end
end
assert(found, 'Fann ikkje boksen');
t0     = buf(end,1);
x0     = mean(buf(:,2) - vBelt*(buf(:,1) - t0));
tPick  = t0 + (cfg.xPick - x0)/vBelt;
tStart = tPick + plan.tau(1) - par.tLag;

tNow = stampSec(receive(sub, 5));
fprintf('x0 = %.3f m   t_pick = %.2f s   margin til start = %.2f s (sim)\n', x0, tPick, tStart - tNow);
assert(tStart - tNow > 0.3, 'For seint: banen rekk ikkje å starte');
sendPlan(pub, msg, plan, tStart);

tAttach = tPick + 0.10;
while true
    js = receive(sub, 5);
    if stampSec(js) >= tAttach, break; end
end
send(attachPub, ros2message(attachPub));
[~,idx] = ismember(jn, string(js.name));
qA = double(js.position(idx))';
TA = double(robot.fkine(qA));
xBoxPred = cfg.xPick + vBelt*(stampSec(js) - tPick);
fprintf('attach ved tau = %.2f s:  TCP x = %.3f   boks x (pred) = %.3f   avvik = %+.1f mm   TCP z = %.3f\n', ...
        stampSec(js) - tPick, TA(1,4), xBoxPred, 1000*(TA(1,4) - xBoxPred), TA(3,4));

qE = readQ();  TE = double(robot.fkine(qE));
fprintf('TCP slutt: x = %.4f   y = %.4f   z = %.4f  (basekoordinatar)\n', TE(1,4), TE(2,4), TE(3,4));

tEnd = tPick + plan.tau(end) + 0.5;
while stampSec(receive(sub, 5)) < tEnd, end

moveJ(pub, msg, readQ, Q.above, vmax, 0.1);                 
moveL(pub, msg, readQ, robot, P.above, P.place, vPlace, 0.1); 
pause(0.5);
send(detachPub, ros2message(detachPub));                     
pause(1);
moveL(pub, msg, readQ, robot, P.place, P.above, vPlace, 0.1); 
moveJ(pub, msg, readQ, Q.wait, vmax, 0.1);                   
fprintf('Sequens done\n');


%% =====Functions=====

function q = local_readQ(sub, jn)
    js = receive(sub, 5);
    [~,idx] = ismember(jn, string(js.name));
    q = double(js.position(idx))';          
end

function ok = waitForPose(readQ, qGoal, tolDeg, timeoutS)
    if nargin < 3, tolDeg = 0.5; end
    if nargin < 4, timeoutS = 60; end
    t0 = tic;
    while toc(t0) < timeoutS
        err = max(abs(rad2deg(readQ() - qGoal)));
        if err < tolDeg
            ok = true;  return
        end
    end
    ok = false;
    warning('waitForPose: nådde ikkje målet innan %.0f s (maks feil %.2f deg)', timeoutS, err);
end

function moveJ(pub, msg, readQ, qGoal, vmax, tolDeg)
    q0 = readQ();
    dq = abs(qGoal - q0);
    T  = max(2, 1.875*max(dq./vmax)/0.8);        
    dt = 0.1;
    t  = (0:dt:T)';
    if t(end) < T, t(end+1) = T; end
    [q, qd] = jtraj(q0, qGoal, t);               
    n   = numel(t) - 1;                         
    pts = repmat(ros2message("trajectory_msgs/JointTrajectoryPoint"), n, 1);
    for k = 1:n
        tk = t(k+1);
        pts(k).positions     = q(k+1,:);
        pts(k).velocities    = qd(k+1,:);
        pts(k).accelerations = zeros(1,0);
        pts(k).effort        = zeros(1,0);
        pts(k).time_from_start.sec     = int32(floor(tk));
        pts(k).time_from_start.nanosec = uint32(min(round((tk-floor(tk))*1e9), 999999999));
    end
    msg.points = pts;
    send(pub, msg);
    waitForPose(readQ, qGoal, tolDeg, 120);
end

function [xc, yc, ok] = detectBox(img, cam)

    ok = false;  xc = NaN;  yc = NaN;
    c0   = (cam.W+1)/2;   r0 = (cam.H+1)/2;         
    half = round(0.18/cam.s);                       
    cols = round(c0)-half : round(c0)+half;
    roi  = double(img(:, cols));

    ref = median(roi(:, [1:4, end-3:end]), 2);     
    bw  = abs(roi - ref) > 25;             
    bw = bwareaopen(bw, 200);
    st = regionprops(bw, 'Area', 'BoundingBox', 'Centroid');
    st = st([st.Area] > 300 & [st.Area] < 1500);
    if isempty(st), return; end
    [~,i] = max([st.Area]);
    bb = st(i).BoundingBox;                          

    if st(i).Centroid(2) < r0                         
        xEdge = cam.pos(1) + (r0 - bb(2)) * cam.s;
        xc = xEdge - cam.box/2;
    else                                              
        xEdge = cam.pos(1) + (r0 - (bb(2)+bb(4))) * cam.s;
        xc = xEdge + cam.box/2;
    end
    
    cx = cols(1) - 1 + st(i).Centroid(1);
    yc = cam.pos(2) - (cx - c0) * cam.s;              
    ok = true;
end

function t = stampSec(m)
    t = double(m.header.stamp.sec) + 1e-9*double(m.header.stamp.nanosec);   
end

function plan = planPick(robot, cfg, par, vBelt, qStart)

    ta  = par.tc - par.Tc;
    tl0 = par.dwell;   tl1 = par.dwell + par.Tl;
    tau = (ta : par.dt : tl1 + par.Tdec)';
    xBox  = @(t) cfg.xPick + vBelt*t;
    xc    = xBox(par.tc);
    xLift = xBox(tl1);
    zTop  = cfg.zBoxTop;
    n = numel(tau);   X = zeros(n,1);   Z = zeros(n,1);
    for i = 1:n
        t = tau(i);
        if t < par.tc                                   
            X(i) = hermiteQ(t - ta, par.Tc, par.xWait, 0, xc, vBelt);
        elseif t < tl1                                  
            X(i) = xBox(t);
        else                                            
            X(i) = hermiteQ(t - tl1, par.Tdec, xLift, vBelt, xLift + vBelt*par.Tdec/2, 0);
        end
        if t < -par.Td                                 
            Z(i) = zTop + cfg.hHover;
        elseif t < 0                                   
            Z(i) = zTop + cfg.hHover*(1 - minJerk((t + par.Td)/par.Td));
        elseif t < tl0                                 
            Z(i) = zTop;
        else                                            
            Z(i) = zTop + par.hLift*minJerk((t - tl0)/par.Tl);
        end
    end

    q = zeros(n,6);   qPrev = qStart;
    for i = 1:n
        T  = transl(X(i), cfg.yBelt, Z(i)) * trotx(pi);
        qi = robot.ikine(T, 'q0', qPrev, 'ilimit', 200, 'rlimit', 100);
        qi = qPrev + atan2(sin(qi - qPrev), cos(qi - qPrev));   
        q(i,:) = qi;   qPrev = qi;
    end
    qd = gradient(q', par.dt)';                        
    plan.tau = tau;  plan.X = X;  plan.Z = Z;  plan.q = q;  plan.qd = qd;
end

function s = minJerk(s)
    s = min(max(s,0),1);
    s = 10*s.^3 - 15*s.^4 + 6*s.^5;
end

function x = hermiteQ(t, T, x0, v0, x1, v1)

    s = min(max(t/T,0),1);
    h00 = 1 - 10*s^3 + 15*s^4 - 6*s^5;   h10 = s - 6*s^3 + 8*s^4 - 3*s^5;
    h01 = 10*s^3 - 15*s^4 + 6*s^5;       h11 = -4*s^3 + 7*s^4 - 3*s^5;
    x = h00*x0 + h10*T*v0 + h01*x1 + h11*T*v1;
end

function sendPlan(pub, msg, plan, tStart)
    t = plan.tau - plan.tau(1);
    n = numel(t) - 1;
    pts = repmat(ros2message("trajectory_msgs/JointTrajectoryPoint"), n, 1);
    for k = 1:n
        tk = t(k+1);
        pts(k).positions     = plan.q(k+1,:);
        pts(k).velocities    = plan.qd(k+1,:);
        pts(k).accelerations = zeros(1,0);
        pts(k).effort        = zeros(1,0);
        pts(k).time_from_start.sec     = int32(floor(tk));
        pts(k).time_from_start.nanosec = uint32(min(round((tk-floor(tk))*1e9), 999999999));
    end
    pts(n).velocities = zeros(1,6);         
    msg.header.stamp.sec     = int32(floor(tStart));
    msg.header.stamp.nanosec = uint32(min(round((tStart-floor(tStart))*1e9), 999999999));
    msg.points = pts;
    send(pub, msg);
end

function moveL(pub, msg, readQ, robot, pA, pB, vTcp, tolDeg)

    L  = norm(pB - pA);
    T  = max(1, 1.875*L/vTcp);                 
    dt = 0.02;
    t  = (0:dt:T)';
    s  = minJerk(t/T);
    Pp = pA + s.*(pB - pA);                    

    q = zeros(numel(t),6);   qPrev = readQ();
    for i = 1:numel(t)
        Ti = transl(Pp(i,:)) * trotx(pi);
        qi = robot.ikine(Ti, 'q0', qPrev, 'ilimit', 200, 'rlimit', 100);
        qi = qPrev + atan2(sin(qi - qPrev), cos(qi - qPrev));
        q(i,:) = qi;   qPrev = qi;
    end
    qd = gradient(q', dt)';

    n   = numel(t) - 1;
    pts = repmat(ros2message("trajectory_msgs/JointTrajectoryPoint"), n, 1);
    for k = 1:n
        tk = t(k+1);
        pts(k).positions     = q(k+1,:);
        pts(k).velocities    = qd(k+1,:);
        pts(k).accelerations = zeros(1,0);
        pts(k).effort        = zeros(1,0);
        pts(k).time_from_start.sec     = int32(floor(tk));
        pts(k).time_from_start.nanosec = uint32(min(round((tk-floor(tk))*1e9), 999999999));
    end
    pts(n).velocities = zeros(1,6);            
    msg.points = pts;
    send(pub, msg);
    waitForPose(readQ, q(end,:), tolDeg, 60);
end
