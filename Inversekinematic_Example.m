
%% =========================================================
% INVERSKINEMATIKK
%
% FORMAL:
% 1. Definere robotarmen med DH-parametrar
% 2. Definere sugekoppen (TCP)
% 3. Legge inn onskja posisjon og orientering
% 4. Berekn dei seks leddvinklane
% 5. Kontrollere resultatet med framoverkinematikk
% 6. Visualisere robotarmen i 3D
%
% Peter Corke Robotics Toolbox
%% =========================================================

clear;
clc;
close all;

%% =========================================================
% STEG 1 - DEFINER ROBOTARMEN
%
% Same DH-parametrar som i oppgave 2.i.1 og 2.i.2.
% Lengder i meter, vinklar i radianar.
%% =========================================================

L(1) = Link('d',0.2,'a',0,'alpha',pi/2);
L(2) = Link('d',0,'a',-0.6,'alpha',0);
L(3) = Link('d',0,'a',-0.6,'alpha',0);
L(4) = Link('d',0.2,'a',0,'alpha',pi/2);
L(5) = Link('d',0.1,'a',0,'alpha',-pi/2);
L(6) = Link('d',0.1,'a',0,'alpha',0);

robot = SerialLink(L,'name','ELE208 robotarm');

%% =========================================================
% STEG 2 - DEFINER SUGEKOPPEN (TCP)
%
% Avstanden fra siste DH-ramme til kontaktflata
% pa eska er 0.196 meter.
%
% Vi antar at griparen er langs lokal positiv z-akse.
%% =========================================================

tool_length = 0.196;

robot.tool = transl(0,0,tool_length);

%% =========================================================
% STEG 3 - ONSKJA POSISJON OG ORIENTERING
%
% Sugekoppen skal treffe toppen av eska: (eksmepelverdiar)
%
% x = 0.30 m
% y = 1.00 m
% z = 0.66 m
%
% Sugekoppane skal vende rett ned.
%
% Rotasjon 180 grader rundt x-aksen gjer at
% lokal z-akse peikar nedover.
%% =========================================================

x = 0.30;
y = 1.00;
z = 0.66;

% Onskja orientering
roll  = pi;
pitch = 0;
yaw   = 0;

% Transformasjonsmatrise for TCP
T_desired = transl(x,y,z) * ...
            trotz(yaw) * troty(pitch) * trotx(roll);

disp('========================================')
disp('ONSKJA TCP-POSISJON OG ORIENTERING')
disp('========================================')
disp(T_desired)


%% STEG 4 - BEREKN INVERSKINEMATIKKEN

% Bruk fleire startposisjonar for a finne ei loysing

rng(1);

best_error = inf;
q_inverse = [];

% Prov 100 forskjellige startposisjonar
for i = 1:100

    if i == 1
        q_start = zeros(1,6);
    else
        q_start = -pi + 2*pi*rand(1,6);
    end

    try
        q_candidate = robot.ikine(T_desired, ...
            'q0',q_start, ...
            'ilimit',1000, ...
            'rlimit',200);

        if isempty(q_candidate) || ...
                any(~isfinite(q_candidate))
            continue
        end

        T_candidate = double(robot.fkine(q_candidate));

        candidate_error = norm( ...
            double(T_desired)-T_candidate,'fro');

        if candidate_error < best_error
            best_error = candidate_error;
            q_inverse = q_candidate;
        end

        % Stopp dersom loysing er tilstrekkeleg noyaktig
        if best_error < 1e-6
            break
        end

    catch
        % Prov neste startposisjon
    end
end

if isempty(q_inverse) || best_error > 1e-3
    error(['Ingen tilstrekkeleg noyaktig IK-loysing funnen. ', ...
        'Kontroller TCP, orientering og robotmodell.']);
end

disp('========================================')
disp('BEREKNA LEDDVINKLAR')
disp('========================================')

disp('Leddvinklar i radianar:')
disp(q_inverse)

disp('Leddvinklar i grader:')
disp(q_inverse*180/pi)


%% =========================================================
% STEG 5 - KONTROLLER RESULTATET
%
% Vi bruker framoverkinematikk for a kontrollere
% at dei berekna leddvinklane gir rett TCP-posisjon
% og orientering.
%% =========================================================

T_check = double(robot.fkine(q_inverse));
T_target = double(T_desired);

% Onskja TCP-posisjon
P_desired = T_target(1:3,4);

% Oppnadd TCP-posisjon
P_actual = T_check(1:3,4);

% Posisjonsavvik i meter
position_error = norm(P_desired-P_actual);

% Orienteringsavvik i grader
R_desired = T_target(1:3,1:3);
R_actual = T_check(1:3,1:3);

R_error = R_desired' * R_actual;

cos_angle = (trace(R_error)-1)/2;
cos_angle = max(-1,min(1,cos_angle));

orientation_error = acos(cos_angle)*180/pi;

disp('========================================')
disp('KONTROLL AV RESULTAT')
disp('========================================')

disp('Onskja TCP-posisjon [x y z] (m):')
disp(P_desired')

disp('Oppnadd TCP-posisjon [x y z] (m):')
disp(P_actual')

fprintf('Posisjonsavvik: %.6e m\n',position_error);
fprintf('Orienteringsavvik: %.6e grader\n',orientation_error);

%% =========================================================
% STEG 6 - VURDER RESULTATET
%
% Vi krev eit lite posisjons- og orienteringsavvik.
%% =========================================================

position_tolerance = 1e-3;  % 1 mm
orientation_tolerance = 0.1; % 0.1 grader

if position_error < position_tolerance && ...
        orientation_error < orientation_tolerance

    disp('RESULTAT: IK-loysinga er godkjend.')

else

    warning('IK-loysinga har for stort avvik.');
end

%% =========================================================
% STEG 7 - VIS ROBOTARMEN I 3D
%
% Figuren viser robotarmen i den berekna stillinga.
%% =========================================================

figure;
robot.plot(q_inverse);
title('ELE208 - Inverskinematikk ved pickup');
