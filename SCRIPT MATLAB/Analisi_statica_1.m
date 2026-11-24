%% Analisi Statica ABB IRB 120 (RRRRRR)
clear all; 
close all;
clc;

alpha = [-pi/2  0  -pi/2  pi/2  -pi/2  0];
a     = [0 0.27 0.07 0 0 0];
d     = [0.29 0 0 0.302 0 0.072];

th1 = 0;
th2 = -pi/2;
th3 = 0;
th4 = 0;
th5 = 0;
th6 = 0;

T56 = [cos(th6), -sin(th6)*cos(alpha(6)),  sin(th6)*sin(alpha(6)),  a(6)*cos(th6);
       sin(th6),  cos(th6)*cos(alpha(6)), -cos(th6)*sin(alpha(6)),  a(6)*sin(th6);
       0,         sin(alpha(6)),           cos(alpha(6)),           d(6);
       0,         0,                       0,                       1];

T45 = [cos(th5), -sin(th5)*cos(alpha(5)),  sin(th5)*sin(alpha(5)),  a(5)*cos(th5);
       sin(th5),  cos(th5)*cos(alpha(5)), -cos(th5)*sin(alpha(5)),  a(5)*sin(th5);
       0,         sin(alpha(5)),           cos(alpha(5)),           d(5);
       0,         0,                       0,                       1];
   
T34 = [cos(th4), -sin(th4)*cos(alpha(4)),  sin(th4)*sin(alpha(4)),  a(4)*cos(th4);
       sin(th4),  cos(th4)*cos(alpha(4)), -cos(th4)*sin(alpha(4)),  a(4)*sin(th4);
       0,         sin(alpha(4)),           cos(alpha(4)),           d(4);
       0,         0,                       0,                       1];

T23 = [cos(th3), -sin(th3)*cos(alpha(3)),  sin(th3)*sin(alpha(3)),  a(3)*cos(th3);
       sin(th3),  cos(th3)*cos(alpha(3)), -cos(th3)*sin(alpha(3)),  a(3)*sin(th3);
       0,         sin(alpha(3)),           cos(alpha(3)),           d(3);
       0,         0,                       0,                       1];

T12 = [cos(th2), -sin(th2)*cos(alpha(2)),  sin(th2)*sin(alpha(2)),  a(2)*cos(th2);
       sin(th2),  cos(th2)*cos(alpha(2)), -cos(th2)*sin(alpha(2)),  a(2)*sin(th2);
       0,         sin(alpha(2)),           cos(alpha(2)),           d(2);
       0,         0,                       0,                       1];

T01 = [cos(th1), -sin(th1)*cos(alpha(1)),  sin(th1)*sin(alpha(1)),  a(1)*cos(th1);
       sin(th1),  cos(th1)*cos(alpha(1)), -cos(th1)*sin(alpha(1)),  a(1)*sin(th1);
       0,         sin(alpha(1)),           cos(alpha(1)),           d(1);
       0,         0,                       0,                       1];

%R = cell(1,6);
R{1} = T01(1:3,1:3);
R{2} = T12(1:3,1:3);
R{3} = T23(1:3,1:3);
R{4} = T34(1:3,1:3);
R{5} = T45(1:3,1:3);
R{6} = T56(1:3,1:3);

%P = cell(1,6);
P{1} = T01(1:3,4);
P{2} = T12(1:3,4);
P{3} = T23(1:3,4);
P{4} = T34(1:3,4);
P{5} = T45(1:3,4);
P{6} = T56(1:3,4);

m_payload = 0.30;  
g0 = 9.81;        

%f = cell(1,7);
%n = cell(1,7);
f{7} = [0; 0; m_payload*g0]; 
n{7} = [0; 0; 0];

for i = 6:-1:1
    f{i} = R{i} * f{i+1};
    n{i} = R{i} * n{i+1} + cross(P{i}, f{i});
end

tau1 = n{1};
tau2 = n{2};
tau3 = n{3};
tau4 = n{4};
tau5 = n{5};
tau6 = n{6};
fprintf("Valore dei momenti ai giunti (vettore n{i}) [N*m]: \n\n");
fprintf("Giunto 1: [%.4f, %.4f, %.4f] \n", tau1(1), tau1(2), tau1(3));
fprintf("Giunto 2: [%.4f, %.4f, %.4f] \n", tau2(1), tau2(2), tau2(3));
fprintf("Giunto 3: [%.4f, %.4f, %.4f] \n", tau3(1), tau3(2), tau3(3));
fprintf("Giunto 4: [%.4f, %.4f, %.4f] \n", tau4(1), tau4(2), tau4(3));
fprintf("Giunto 5: [%.4f, %.4f, %.4f] \n", tau5(1), tau5(2), tau5(3));
fprintf("Giunto 6: [%.4f, %.4f, %.4f] \n", tau6(1), tau6(2), tau6(3));
