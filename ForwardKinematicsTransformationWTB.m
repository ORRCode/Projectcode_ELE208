clear
clc

syms q1 q2 q3 q4 q5 q6 real
L1 = 0.6; L2 = 0.6; L3 = 0.2; L4 = 0.1; L5 = 0.1; h = 0.2;

% DH-tabell: [theta, d, a, alpha]
DH = [ q1   h   0    pi/2;
       q2   0    -L1   0;
       q3   0    -L2   0;
       q4   L3   0   pi/2;
       q5   L4    0    -pi/2;
       q6   L5    0    0 ];

% Transformasjon for kvart ledd
A = cell(1,6);
for i = 1:6
    A{i} = dhTransform(DH(i,1), DH(i,2), DH(i,3), DH(i,4));
end

% Kumulative transformasjonar T_0^i
T = cell(1,6);
T{1} = A{1};
for i = 2:6
    T{i} = simplify(T{i-1} * A{i});
end


T04 = simplify(T{4})
p04 = T04(1:3,4)
R04 = T04(1:3,1:3)

%% Samanlikning: manuell T04 mot fkine (Peter Corke's Robotics Toolbox)

% (standard DH, same konvensjon som dhTransform)
Lk(1) = Revolute('d', h,  'a',  0,   'alpha',  pi/2);
Lk(2) = Revolute('d', 0,  'a', -L1,  'alpha',  0);
Lk(3) = Revolute('d', 0,  'a', -L2,  'alpha',  0);
Lk(4) = Revolute('d', L3, 'a',  0,   'alpha',  pi/2);
robot = SerialLink(Lk, 'name', 'robot');

% Testkonfigurasjonar (rad = ein konfigurasjon, kolonnar = q1..q4)
qtest = [ 0     0     0     0;
          pi/4  pi/6 -pi/3  pi/2;
         -pi/2  pi/3  pi/4 -pi/6;
          1.0  -0.5   0.7   2.0 ];

maxFeil = 0;
for k = 1:size(qtest,1)
    qk = qtest(k,:);

    % Manuell (symbolsk) T04 med tal innsett
    T_man = double(subs(T04, [q1 q2 q3 q4], qk));

    % fkine frå toolbox
    T_fk = robot.fkine(qk);
    if isa(T_fk, 'SE3')
        T_fk = T_fk.T;
    end

    feil = max(abs(T_man(:) - T_fk(:)));
    maxFeil = max(maxFeil, feil);
    fprintf('Konfigurasjon %d: maks avvik = %.2e\n', k, feil);
end

if maxFeil < 1e-10
    disp('OK: manuell T04 og fkine gir same resultat.')
else
    disp('FEIL: avvik mellom manuell T04 og fkine, sjekk DH-tabellen.')
end

%% Funksjon for å sette DH-parameter inn i standard matrise
function A = dhTransform(theta, d, a, alpha)
    A = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
          sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
          0,           sin(alpha),             cos(alpha),            d;
          0,           0,                      0,                     1 ];
end