% Workspace e Path Planning ABB IRB 120 (RRRRRR)
clear all;
close all;
clc;
%% Definizione dei parametri DH e limiti dei giunti
alpha = [-90, 0, -90, 90, -90, 0] * pi/180; % Angoli alpha in radianti
a = [0 0.27 0.07 0 0 0]; % Lunghezze dei collegamenti
d = [0.29 0 0 0.302 0 0.072]; % Offset dei giunti
th_min = [-165, -110, -110, -160, -120, -400] * pi/180; % Minimi dei giunti
th_max = [165, 110, 70, 160, 120, 400] * pi/180; % Massimi dei giunti
%% Calcolo del workspace
p1 = pi/9; % Passo di variazione
p2 = pi/2;
k = 1; % Contatore delle posizioni
xH = []; yH = []; zH = []; % Inizializzazione
for th1 = th_min(1):p1:th_max(1)
T01 = [cos(th1), -sin(th1)*cos(alpha(1)), sin(th1)*sin(alpha(1)), a(1)*cos(th1);
sin(th1), cos(th1)*cos(alpha(1)), -cos(th1)*sin(alpha(1)), a(1)*sin(th1);
0, sin(alpha(1)), cos(alpha(1)), d(1);
0, 0, 0, 1];
for th2 = th_min(2):p1:th_max(2)
T12 = [cos(th2), -sin(th2)*cos(alpha(2)), sin(th2)*sin(alpha(2)), a(2)*cos(th2);
sin(th2), cos(th2)*cos(alpha(2)), -cos(th2)*sin(alpha(2)), a(2)*sin(th2);
0, sin(alpha(2)), cos(alpha(2)), d(2);
0, 0, 0, 1];
for th3 = th_min(3):p1:th_max(3)
T23 = [cos(th3), -sin(th3)*cos(alpha(3)), sin(th3)*sin(alpha(3)), a(3)*cos(th3);
sin(th3), cos(th3)*cos(alpha(3)), -cos(th3)*sin(alpha(3)), a(3)*sin(th3);
0, sin(alpha(3)), cos(alpha(3)), d(3);
0, 0, 0, 1];
for th4 = th_min(4):p2:th_max(4)
T34 = [cos(th4), -sin(th4)*cos(alpha(4)), sin(th4)*sin(alpha(4)), a(4)*cos(th4);
sin(th4), cos(th4)*cos(alpha(4)), -cos(th4)*sin(alpha(4)), a(4)*sin(th4);
0, sin(alpha(4)), cos(alpha(4)), d(4);
0, 0, 0, 1];
for th5 = th_min(5):p2:th_max(5)
T45 = [cos(th5), -sin(th5)*cos(alpha(5)), sin(th5)*sin(alpha(5)), a(5)*cos(th5);
sin(th5), cos(th5)*cos(alpha(5)), -cos(th5)*sin(alpha(5)), a(5)*sin(th5);
0, sin(alpha(5)), cos(alpha(5)), d(5);
0, 0, 0, 1];
for th6 = th_min(6):p2:th_max(6)
T56 = [cos(th6), -sin(th6)*cos(alpha(6)), sin(th6)*sin(alpha(6)), a(6)*cos(th6);
sin(th6), cos(th6)*cos(alpha(6)), -cos(th6)*sin(alpha(6)), a(6)*sin(th6);
0, sin(alpha(6)), cos(alpha(6)), d(6);
0, 0, 0, 1];
T06 = T01 * T12 * T23 * T34 * T45 * T56;
H = T06 * [0; 0; 0; 1];
xH(k) = H(1); yH(k) = H(2); zH(k) = H(3);
k = k + 1;
end
end
end
end
end
end
%% Visualizzazione del workspace
figure(1);
plot3(xH, yH, zH, 'b.', 'DisplayName', 'Workspace');
hold on;
title('Workspace 3D del robot ABB IRB 120');
xlabel('X [m]'); ylabel('Y [m]'); zlabel('Z [m]');
grid on; axis equal;
%% Pianificazione del percorso tramite spline cubica
A = [0.3, 0, 0.4];   % Punto A [x, y, z] 
B = [0, 0.3, 0.4];   % Punto B [x, y, z]  
num_points = 100;    % Numero di punti nel percorso
time = linspace(0, 10, num_points); % Tempi uniformemente distribuiti
% Punti chiave per la spline
key_times = [0, 10]; % Tempi associati ai punti chiave
key_points = [A; B]; % Punti chiave nello spazio operativo
% Interpolazione spline cubica per ogni coordinata
spline_x = spline(key_times, [0, key_points(:, 1)', 0]);
spline_y = spline(key_times, [0, key_points(:, 2)', 0]);
spline_z = spline(key_times, [0, key_points(:, 3)', 0]);
% Valori interpolati
path = [ppval(spline_x, time)', ppval(spline_y, time)', ppval(spline_z, time)'];
% Calcolo degli angoli di giunto usando la cinematica inversa
theta_path = zeros(num_points, 6);
for i = 1:num_points
x = path(i, 1); y = path(i, 2); z = path(i, 3);
theta1 = atan2(y, x);
r = sqrt(x^2 + y^2);
cos_theta2 = (r^2 - a(2)^2 - a(3)^2) / (2 * a(2) * a(3));
theta2 = atan2(sqrt(1 - cos_theta2^2), cos_theta2);
theta3 = z - d(1) - d(4);
theta4 = 0; % Configurazione arbitraria
theta5 = 0; % Configurazione arbitraria
theta6 = 0; % Configurazione arbitraria
theta_path(i, :) = max(min([theta1, theta2, theta3, theta4, theta5, theta6], th_max), th_min);
end
% Visualizzazione del percorso nello spazio di lavoro
plot3(path(:, 1), path(:, 2), path(:, 3), 'r-', 'LineWidth', 2, 'DisplayName', 'Percorso');
legend;
%% Grafici cinematici per posizione, velocità e accelerazione
num_joints = size(theta_path, 2);
for joint = 1:num_joints
theta = theta_path(:, joint);
vel = gradient(theta, time);
acc = gradient(vel, time);
% Creazione dei grafici per ciascun giunto
figure;
subplot(3, 1, 1);
plot(time, theta, 'LineWidth', 1.5);
title(['Posizione Giunto ', num2str(joint)]);
xlabel('Tempo [s]'); ylabel('Posizione [rad]'); grid on;
subplot(3, 1, 2);
plot(time, vel, 'LineWidth', 1.5);
title(['Velocità Giunto ', num2str(joint)]);
xlabel('Tempo [s]'); ylabel('Velocità [rad/s]'); grid on;
subplot(3, 1, 3);
plot(time, acc, 'LineWidth', 1.5);
title(['Accelerazione Giunto ', num2str(joint)]);
xlabel('Tempo [s]'); ylabel('Accelerazione [rad/s^2]'); grid on;
end