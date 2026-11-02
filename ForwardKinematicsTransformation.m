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

T01 = A{1};
T12 = A{2};
T23 = A{3};
T34 = A{4};
T45 = A{5};
T56 = A{6};

T01 = T{1};
T02 = T{2};
T03 = T{3};
T04 = simplify(T{4})
T06 = T{6};
p04 = T04(1:3,4)
R04 = T04(1:3,1:3)

%% Funksjon for å sette DH-parameter inn i standard matrise
function A = dhTransform(theta, d, a, alpha)
    A = [ cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha), a*cos(theta);
          sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
          0,           sin(alpha),             cos(alpha),            d;
          0,           0,                      0,                     1 ];
end