
%% UR10e PICK AND PLACE - MATLAB + ROS2 + GAZEBO
% Peter Corke Robotics Toolbox
%
% Flyt:
% HOME -> READY -> WAIT_BOX -> TRACK -> VACUUM -> LIFT
% -> RETRACT -> ROTATE 180 -> PLACE -> RELEASE -> HOME
%
% Alle posisjonar er i meter, leddvinklar i radianar.
% Endre innstillingane i del 1, ikkje hjelpefunksjonane.

clear; clc;
setenv("ROS_DOMAIN_ID","67");

%% 1. INNSTILLINGAR
% ------------------------------------------------------------
% Desse verdiane kan du justere for aa endre bevegelsane.

joints = {'shoulder_pan_joint','shoulder_lift_joint', ...
          'elbow_joint','wrist_1_joint', ...
          'wrist_2_joint','wrist_3_joint'};

% KALIBRERT HOME - IKKJE ENDRE
home = [-5.590227783399992, -1.4988414790997258, ...
         1.6432331713008754, -1.7151880190984519, ...
         4.712388980799992, -0.877838803199999];

homeTCP = [0.45 0.60 0.45];
ready   = [0.45 1.00 0.57];
retract = [0.30 0.65 0.70];

tcpOffset = 0.143025;
tcpQuat = [0 1 0 0];            % w, x, y, z

% TRACKING - IKKJE ENDRE FORELOPIG
% Kvar rad er eit TCP-punkt [X Y Z].
trackTime = [0.20 0.35 0.50 0.65 0.80];
trackXYZ = [0.450 1.000 0.570;
            0.355 1.000 0.570;
            0.310 1.000 0.510;
            0.265 1.000 0.450;
            0.220 1.000 0.450];

trackDt = 0.02;                % 20 ms mellom punkta
triggerX = 0.53;               % Start tracking naar boks passerer X
boxY = 1.00;                   % Transportbandet si Y-linje

% Tider for bevegelsar (sekund)
readyTime   = 4.0;
liftTime    = 2.5;             % Raskare loft etter vakuum
retractTime = 4.0;
rotateTime  = 8.0;
placeTime   = 5.0;
retreatTime = 4.0;
homeTime    = 7.0;

liftZ = 0.65;                  % Hogd etter plukk
vacuumWait = 0.3;              % Vent etter vakuumfesting
rotation = pi;                 % 180 grader med baseleddet

% PALL - DESSE VERDIANE MAA VERIFISERAST I GAZEBO
palletXY = [0.00 -0.80];       % Forventa pallmidtpunkt
palletTolerance = 0.45;       % Tillate XY-avvik
placeZ = 0.344;               % TCP-hogd ved slepp
retreatHeight = 0.20;         % Loft etter slepp

% Hastigheitsgrenser [rad/s]
speedLimit = [2.094395 2.094395 3.141593 ...
              3.141593 3.141593 3.141593];

jointTolerance = 0.10;         % Tillate leddavvik [rad]
maxIKStep = pi/2;              % Maks 90 grader per IK-steg

%% 2. KOPLE MATLAB TIL ROS2
% ------------------------------------------------------------
% Subscriber: les data fraa Gazebo.
% Publisher: sender kommandoar til transportband og vakuum.
% Action client: sender leddbaner til UR10e.

disp("Koplar til ROS2 ...");

node = ros2node("/matlab_pick_and_place");

jointSub = ros2subscriber(node,"/joint_states", ...
    "sensor_msgs/JointState");

boxSub = ros2subscriber(node,"/detected_package", ...
    "geometry_msgs/PointStamped");

beltPub = ros2publisher(node,"/conveyor/run","std_msgs/Bool");
attachPub = ros2publisher(node,"/vacuum/attach","std_msgs/Empty");
detachPub = ros2publisher(node,"/vacuum/detach","std_msgs/Empty");
activePub = ros2publisher(node,"/vacuum/active","std_msgs/Bool");

client = ros2actionclient(node, ...
    "/scaled_joint_trajectory_controller/follow_joint_trajectory", ...
    "control_msgs/FollowJointTrajectory");

waitForServer(client);

% Stoppar bandet automatisk dersom programmet avsluttar.
cleanup = onCleanup(@() stopBelt(beltPub));

belt(beltPub,false);
vacuum(detachPub,activePub,false);

%% 3. OPPRETT UR10e-MODELLEN
% ------------------------------------------------------------
% DH-parametrane beskriv geometrien til roboten.
% fkine: leddvinklar -> TCP-posisjon
% ikcon: oenskt TCP-posisjon -> leddvinklar

DH = [0.1807   0        pi/2;
      0       -0.6127   0;
      0       -0.57155  0;
      0.17415  0        pi/2;
      0.11985  0       -pi/2;
      0.11655  0        0];

for i = 1:6
    L(i) = Link('d',DH(i,1),'a',DH(i,2), ...
                'alpha',DH(i,3),'standard');
end

robot = SerialLink(L,'name','UR10e');

for i = 1:6
    robot.links(i).qlim = [-2*pi 2*pi];
end

%% 4. KALIBRER ROBOTEN MOT GAZEBO
% ------------------------------------------------------------
% Vi kjenner HOME i baade leddvinklar og world-koordinatar.
% Dermed kan vi finne transformasjonen world -> robotbase.

q = readJoints(jointSub,joints);

if max(abs(angleDiff(q,home))) > 0.08
    error("Roboten maa staa i HOME ved oppstart.");
end

TtcpTool = transl(0,0,-tcpOffset);
ThomeTCP = transl(homeTCP)*quatTransform(tcpQuat);

TworldBase = (ThomeTCP*TtcpTool)/double(robot.fkine(home));
TbaseWorld = inv(TworldBase);

% Samle transformasjonar og IK-grenser i ein struktur.
kin.robot = robot;
kin.TworldBase = TworldBase;
kin.TbaseWorld = TbaseWorld;
kin.TtcpTool = TtcpTool;
kin.tcpOffset = tcpOffset;
kin.tcpQuat = tcpQuat;
kin.maxStep = maxIKStep;

%% 5. MOVE_READY
% ------------------------------------------------------------
% Flytt roboten til startposisjonen over transportbandet.

disp("STATE: MOVE_READY");

qReady = ikXYZ(kin,ready,q);
moveRobot(client,jointSub,joints,qReady,readyTime, ...
    "MOVE_READY",speedLimit,jointTolerance);

%% 6. REKN UT TRACKING-BANA
% ------------------------------------------------------------
% Finn leddvinklar for dei fem kjende TCP-punkta.
% Dette blir gjort FOER bandet startar, slik at roboten
% ikkje brukar tid paa IK medan boksen bevegar seg.

disp("STATE: PREPARE_TRACK");

qTrack = zeros(5,6);
q = readJoints(jointSub,joints);

for i = 1:5
    q = ikXYZ(kin,trackXYZ(i,:),q);
    qTrack(i,:) = q;
end

% PCHIP gir ei jamn leddbane gjennom dei fem punkta.
tNodes = [0 trackTime];
qNodes = [readJoints(jointSub,joints); qTrack];

tTrack = (0:trackDt:trackTime(end))';
trackQ = zeros(numel(tTrack),6);
trackV = zeros(numel(tTrack),6);

for j = 1:6
    trackQ(:,j) = pchip(tNodes,qNodes(:,j),tTrack);
    trackV(:,j) = gradient(trackQ(:,j),tTrack);
end

trackV(1,:) = 0;
trackV(end,:) = 0;

checkSpeed(trackV,speedLimit);

%% 7. START BAND OG VENT PAA BOKS
% ------------------------------------------------------------
% Kamera sender boksen sin posisjon paa /detected_package.
% Vi ventar til boksen kjem fraa X > 0.53 og passerer X=0.53.
% Dette er den fungerande triggeren fraa tidlegare testar.

disp("STATE: WAIT_BOX");
belt(beltPub,true);

seenUpstream = false;

while true
    try
        msg = receive(boxSub,0.2);
    catch
        continue
    end

    if string(msg.header.frame_id) ~= "world"
        continue
    end

    xyz = [double(msg.point.x) ...
           double(msg.point.y) ...
           double(msg.point.z)];

    if any(~isfinite(xyz)) || abs(xyz(2)-boxY)>0.08 ...
            || xyz(1)<0.35 || xyz(1)>0.80
        continue
    end

    if xyz(1)>triggerX
        seenUpstream = true;
    end

    if seenUpstream && xyz(1)<=triggerX
        fprintf("Boks funnen: X=%.3f Y=%.3f Z=%.3f\n",xyz);
        break
    end
end

%% 8. TRACK_AND_LOWER
% ------------------------------------------------------------
% Roboten foelgjer boksen og senkar sugekoppen.
% Vi sender heile bana som ein ROS2-action.

disp("STATE: TRACK_AND_LOWER");

sendTrajectory(client,joints,trackQ,trackV,tTrack, ...
    "TRACK_AND_LOWER");

belt(beltPub,false);
pause(0.2);

%% 9. VACUUM_ATTACH
% ------------------------------------------------------------
% Kontroller faktisk TCP-posisjon etter tracking.
% Kameraavvik blir vist, men vi flyttar ikkje roboten
% ein ekstra gong - det forstyrra plukkinga tidlegare.

disp("STATE: VACUUM_ATTACH");

q = readJoints(jointSub,joints);
tcp = getTCP(kin,q);

fprintf("TCP ved plukk: [%.3f %.3f %.3f]\n",tcp);

if norm(tcp-trackXYZ(end,:)) > 0.025
    error("Roboten naadde ikkje plukkeposisjonen.");
end

try
    msg = receive(boxSub,0.5);

    if string(msg.header.frame_id)=="world"
        box = [double(msg.point.x) ...
               double(msg.point.y) ...
               double(msg.point.z)];

        xyError = norm(tcp(1:2)-box(1:2));
        zError = abs(tcp(3)-box(3));

        fprintf("TCP/boks-avvik: XY=%.3f Z=%.3f m\n", ...
            xyError,zError);

        if xyError>0.06 || zError>0.03
            warning("Boksen er ikkje presist under TCP.");
        end
    end
catch
    disp("Ingen ny kameramaaling ved plukk.");
end

vacuum(attachPub,activePub,true);
pause(vacuumWait);

%% 10. LIFT
% ------------------------------------------------------------
% Loft boksen rett opp til Z=0.65 paa 2.5 sekund.

disp("STATE: LIFT");

liftXYZ = [tcp(1:2) liftZ];

q = readJoints(jointSub,joints);
qLift = ikXYZ(kin,liftXYZ,q);

moveRobot(client,jointSub,joints,qLift,liftTime, ...
    "LIFT",speedLimit,jointTolerance);

%% 11. RETRACT
% ------------------------------------------------------------
% Trekk roboten bort fraa transportbandet foer rotasjon.

disp("STATE: RETRACT");

q = readJoints(jointSub,joints);
qRetract = ikXYZ(kin,retract,q);

moveRobot(client,jointSub,joints,qRetract,retractTime, ...
    "RETRACT",speedLimit,jointTolerance);

%% 12. ROTATE_BASE - 180 GRADER
% ------------------------------------------------------------
% Roter berre ledd 1. Dei andre ledda held same vinkel.
% Kontroller at maalet er innanfor leddgrensa.

disp("STATE: ROTATE_BASE");

q = readJoints(jointSub,joints);
qRotate = q;
qRotate(1) = q(1)+rotation;

if qRotate(1)<robot.links(1).qlim(1) || ...
   qRotate(1)>robot.links(1).qlim(2)
    error("Base-rotasjon utanfor leddgrense.");
end

moveRobot(client,jointSub,joints,qRotate,rotateTime, ...
    "ROTATE_BASE",speedLimit,jointTolerance);

%% 13. KONTROLLER POSISJON OVER PALL
% ------------------------------------------------------------
% Ein 180-graders rotasjon garanterer ikkje at TCP
% hamnar over pallen. Difor kontrollerer vi XY foerst.

disp("STATE: VERIFY_PALLET_POSITION");

q = readJoints(jointSub,joints);
tcp = getTCP(kin,q);

palletError = norm(tcp(1:2)-palletXY);

fprintf("TCP etter rotasjon: [%.3f %.3f %.3f]\n",tcp);
fprintf("Avstand til pallmidtpunkt: %.3f m\n",palletError);

if palletError>palletTolerance
    error("TCP er ikkje over definert pallomraade.");
end

%% 14. PLACE_DOWN
% ------------------------------------------------------------
% Senk rett ned med SAME orientering som etter rotasjon.
% Vi endrar berre world-Z, ikkje X, Y eller orientering.

disp("STATE: PLACE_DOWN");

T = getTCPTransform(kin,q);
T(3,4) = placeZ;

qPlace = ikPose(kin,T,q);

moveRobot(client,jointSub,joints,qPlace,placeTime, ...
    "PLACE_DOWN",speedLimit,jointTolerance);

%% 15. RELEASE
% ------------------------------------------------------------
% Slaa av vakuumet og slepp boksen.

disp("STATE: RELEASE");

vacuum(detachPub,activePub,false);
pause(0.7);

%% 16. RETREAT
% ------------------------------------------------------------
% Loft sugekoppen 20 cm rett opp etter slepp.

disp("STATE: RETREAT");

q = readJoints(jointSub,joints);
T = getTCPTransform(kin,q);
T(3,4) = T(3,4)+retreatHeight;

qRetreat = ikPose(kin,T,q);

moveRobot(client,jointSub,joints,qRetreat,retreatTime, ...
    "RETREAT",speedLimit,jointTolerance);

%% 17. RETURN_HOME
% ------------------------------------------------------------
% Koeyr tilbake til den opphavlege HOME-posisjonen.

disp("STATE: RETURN_HOME");

moveRobot(client,jointSub,joints,home,homeTime, ...
    "RETURN_HOME",speedLimit,jointTolerance);

disp("STATE: DONE");
disp("Pick-and-place-sekvensen er fullfort!");

%% ============================================================
% HJELPEFUNKSJONAR
% Desse treng du normalt ikkje endre.
%% ============================================================

function belt(pub,on)
% Start eller stopp transportbandet.
    msg = ros2message(pub);
    msg.data = logical(on);
    send(pub,msg);
end

function stopBelt(pub)
% Sikker stopp av transportbandet ved avslutning.
    try
        belt(pub,false);
    catch
    end
end

function vacuum(pub,activePub,on)
% Send attach/detach og oppdater vakuumsignalet.
    send(pub,ros2message(pub));
    msg = ros2message(activePub);
    msg.data = logical(on);
    send(activePub,msg);
    fprintf("VACUUM = %d\n",on);
end

function q = readJoints(sub,joints)
% Les dei seks UR10e-ledda i rett rekkje.
    timer = tic;

    while toc(timer)<10
        try
            msg = receive(sub,1);
        catch
            continue
        end

        [found,idx] = ismember(string(joints),string(msg.name));

        if all(found) && numel(msg.position)>=max(idx)
            q = double(msg.position(idx));
            q = q(:)';
            return
        end
    end

    error("Fekk ikkje gyldige joint states.");
end

function T = quatTransform(q)
% Konverter quaternion [w x y z] til 4x4 transformasjon.
    q = double(q(:)')/norm(q);
    w=q(1); x=q(2); y=q(3); z=q(4);

    R = [1-2*(y^2+z^2), 2*(x*y-z*w), 2*(x*z+y*w);
         2*(x*y+z*w), 1-2*(x^2+z^2), 2*(y*z-x*w);
         2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x^2+y^2)];

    T = eye(4);
    T(1:3,1:3) = R;
end

function T = getTCPTransform(kin,q)
% FK: rekn ut TCP-transformasjonen i world.
    T = kin.TworldBase * ...
        double(kin.robot.fkine(q)) * ...
        transl(0,0,kin.tcpOffset);
end

function xyz = getTCP(kin,q)
% Hent berre XYZ fraa TCP-transformasjonen.
    T = getTCPTransform(kin,q);
    xyz = T(1:3,4)';
end

function q = ikXYZ(kin,xyz,qStart)
% IK for oenskt TCP-posisjon med fast plukkeorientering.
    T = transl(xyz)*quatTransform(kin.tcpQuat);
    q = ikPose(kin,T,qStart);
end

function qBest = ikPose(kin,TworldTCP,qStart)
% IK med fleire startgjettingar.
% Vel gyldig loeysing som krev minst leddbevegelse.

    target = kin.TbaseWorld*TworldTCP*kin.TtcpTool;
    robot = kin.robot;

    seeds = qStart;

    for j = 1:6
        delta = zeros(1,6);
        delta(j) = 0.2;
        seeds = [seeds; qStart+delta; qStart-delta]; %#ok<AGROW>
    end

    bestScore = inf;
    qBest = [];

    for i = 1:size(seeds,1)
        try
            candidate = double(robot.ikcon(target,seeds(i,:)));
            candidate = candidate(:)';

            if numel(candidate)~=6 || any(~isfinite(candidate))
                continue
            end

            candidate = qStart+angleDiff(candidate,qStart);
            delta = candidate-qStart;

            if any(abs(delta)>kin.maxStep)
                continue
            end

            % Kontroller leddgrenser.
            insideLimits = true;

            for j = 1:6
                lim = robot.links(j).qlim;
                if candidate(j)<lim(1) || candidate(j)>lim(2)
                    insideLimits = false;
                    break
                end
            end

            if ~insideLimits
                continue
            end

            % Kontroller posisjon og orientering.
            actual = double(robot.fkine(candidate));
            posError = norm(actual(1:3,4)-target(1:3,4));
            rotError = norm(actual(1:3,1:3)- ...
                            target(1:3,1:3),'fro');

            if posError>0.005 || rotError>0.03
                continue
            end

            score = sum(delta.^2);

            if score<bestScore
                bestScore = score;
                qBest = candidate;
            end
        catch
            % Proev neste startgjetting.
        end
    end

    if isempty(qBest)
        error("Ingen gyldig IK-loysing.");
    end
end

function d = angleDiff(a,b)
% Kortaste vinkelforskjell mellom to leddvinklar.
    d = atan2(sin(a-b),cos(a-b));
end

function checkSpeed(v,limits)
% Avbryt dersom ei planlagd leddhastigheit er for hog.
    if any(max(abs(v),[],1)>limits)
        error("Planlagt leddhastigheit er for hog.");
    end
end

function moveRobot(client,sub,joints,target,duration,name,limits,tol)
% Lag ei mjuk jtraj-bane og send ho til ROS2.

    startQ = readJoints(sub,joints);
    t = linspace(0,duration,max(21,ceil(duration/0.05)+1))';

    [q,v] = jtraj(startQ,target,t);

    checkSpeed(v,limits);
    sendTrajectory(client,joints,q,v,t,name);

    actual = readJoints(sub,joints);

    if max(abs(angleDiff(actual,target)))>tol
        error("%s: roboten naadde ikkje maalposisjonen.",name);
    end
end

function sendTrajectory(client,joints,q,v,t,name)
% Konverter MATLAB-bana til ROS2 JointTrajectory.
% Vent paa action-resultatet foer neste bevegelse.

    traj = ros2message("trajectory_msgs/JointTrajectory");
    traj.joint_names = joints;

    n = size(q,1);
    points = repmat(ros2message( ...
        "trajectory_msgs/JointTrajectoryPoint"),1,n);

    for k = 1:n
        pt = ros2message("trajectory_msgs/JointTrajectoryPoint");

        pt.positions = q(k,:);
        pt.velocities = v(k,:);

        sec = floor(t(k));
        ns = round((t(k)-sec)*1e9);

        if ns>=1e9
            sec = sec+1;
            ns = ns-1e9;
        end

        pt.time_from_start.sec = int32(sec);
        pt.time_from_start.nanosec = uint32(ns);

        points(k) = pt;
    end

    traj.points = points;

    goal = ros2message(client);
    goal.trajectory = traj;
    goal.goal_time_tolerance.sec = int32(2);

    handle = sendGoal(client,goal);
    timer = tic;

    while true
        status = getStatus(client,handle);

        if any(status==[4 5 6])
            break
        end

        if toc(timer)>90
            error("%s: tidsavbrot.",name);
        end

        pause(0.02);
    end

    result = getResult(client,handle);

    fprintf("%s -> status=%d\n",name,status);

    if status~=4 || double(result.error_code)~=0
        error("%s: ROS2-action feila (kode %d).", ...
            name,double(result.error_code));
    end
end
