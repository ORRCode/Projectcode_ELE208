
%% =========================================================
% DIFFERENSIALKINEMATIKK
%
% Undersoke samanhengen mellom:
% 1. Hastigheita til dei seks ledda
% 2. Hastigheita til sugekoppen (end-effector)
%
% Brukar Peter Corke Robotics Toolbox.
%% =========================================================

clear;
clc;
close all;

%% STEG 1 - DEFINER ROBOTARMEN
% DH-parametrar

L(1) = Link('d',0.2,'a',0,'alpha',pi/2);
L(2) = Link('d',0,'a',-0.6,'alpha',0);
L(3) = Link('d',0,'a',-0.6,'alpha',0);
L(4) = Link('d',0.2,'a',0,'alpha',pi/2);
L(5) = Link('d',0.1,'a',0,'alpha',-pi/2);
L(6) = Link('d',0.1,'a',0,'alpha',0);

robot = SerialLink(L,'name','ELE208 robotarm');


%% STEG 2 - VEL ROBOTSTILLING
% Vi vel vinklane til dei seks ledda.
% Desse vinklane blir brukte i begge berekningane.

q = [0 pi/4 -pi/4 0 pi/4 0];


%% STEG 3 - BEREKN JACOBIANEN
% Jacobianen koplar saman leddhastigheitane og hastigheita til sugekoppen.
% Formel: v = J * q_dot

J = robot.jacob0(q);

disp('======================================')
disp('JACOBIAN-MATRISE')
disp('======================================')
disp(J)

%% =========================================================
% KJENNER ØNSKJA HASTIGHEIT TIL SUGEKOPPEN OG VIL FINNE NODVENDIGE LEDDHASTIGHEITER.
% Prosjektkrav: minst 0.5 m/s under plukkerorsla.
% Vel 0.5 m/s langs negativ x-retning og ingen rotasjon.
%% =========================================================

% Onskja hastigheit til sugekoppen.
v_desired = [-0.5;
             0;
             0;
             0;
             0;
             0];

% Finn nodvendige leddhastigheiter.
q_dot_B = pinv(J) * v_desired;

disp(' ')
disp('======================================')
disp('BEREKNING B')
disp('======================================')

disp('Onskja sugekopphastigheit:')
disp(v_desired)

disp('Nodvendige leddhastigheiter (rad/s):')
disp(q_dot_B)


%% STEG 4 - KONTROLLER BEREKNING 
%
% Brukar dei berekna leddhastigheitene for a kontrollere om vi oppnar 0.5 m/s.

v_check = J * q_dot_B;

speed_B = norm(v_check(1:3));

error_B = norm(v_desired - v_check);

disp(' ')
disp('======================================')
disp('KONTROLL AV BEREKNING B')
disp('======================================')

fprintf('Onskja hastigheit: %.4f m/s\n',0.5);
fprintf('Oppnadd hastigheit: %.4f m/s\n',speed_B);
fprintf('Avvik: %.6e\n',error_B);


%% STEG 5 - VIS ROBOTARMEN I 3D
% Figuren vis robotstillinga me brukte.

figure;
robot.plot(q);
title('ELE208 - Differensialkinematikk');
