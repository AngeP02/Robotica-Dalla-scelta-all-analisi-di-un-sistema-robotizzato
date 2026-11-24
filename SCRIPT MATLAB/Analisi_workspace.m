% Workspace Robot ABB IRB 120 (6 DOF)

% Pulizia dell'ambiente di lavoro
clear all;
close all;
clc;

% Parametri di Denavit-Hartenberg
d = [0.29 0 0 0.302 0 0.072];        % Offset del giunto lungo l'asse z (m)           
a = [0 0.27 0.07 0 0 0];    % Lunghezza del link tra i giunti (m)
alpha = [-pi/2 0 -pi/2 pi/2 -pi/2 0];  % Angoli di twist in radianti

% Variazioni dei giunti
theta1_min = -165*pi/180;
theta1_max = 165*pi/180;
theta2_min = -110*pi/180;
theta2_max = 110*pi/180;
theta3_min = -110*pi/180;
theta3_max = 70*pi/180;
theta4_min = -160*pi/180;
theta4_max = 160*pi/180;
theta5_min = -120*pi/180;
theta5_max = 120*pi/180;
theta6_min = -400*pi/180;
theta6_max = 400*pi/180;

passo1 = pi/9; %passo
passo2 = pi/2; 
k = 1;

% Ciclo per variare il giunto 1
for th1 = theta1_min:passo1:theta1_max
    % Matrice di trasformazione T01 per il giunto 1
    T01 = [cos(th1), -sin(th1)*cos(alpha(1)), sin(th1)*sin(alpha(1)), a(1)*cos(th1);
           sin(th1), cos(th1)*cos(alpha(1)), -cos(th1)*sin(alpha(1)), a(1)*sin(th1);
           0, sin(alpha(1)), cos(alpha(1)), d(1);
           0, 0, 0, 1];

    % Ciclo per variare il giunto 2
    for th2 = theta2_min:passo1:theta2_max
    % Matrice di trasformazione T12 per il giunto 2
        T12 = [cos(th2), -sin(th2)*cos(alpha(2)), sin(th2)*sin(alpha(2)), a(2)*cos(th2);
               sin(th2), cos(th2)*cos(alpha(2)), -cos(th2)*sin(alpha(2)), a(2)*sin(th2);
               0, sin(alpha(2)), cos(alpha(2)), d(2);
               0, 0, 0, 1];
        
        % Ciclo per variare il giunto 3
        for th3 = theta3_min:passo1:theta3_max
        % Matrice di trasformazione T23 per il giunto 3
            T23 = [cos(th3), -sin(th3)*cos(alpha(3)), sin(th3)*sin(alpha(3)), a(3)*cos(th3);
                   sin(th3), cos(th3)*cos(alpha(3)), -cos(th3)*sin(alpha(3)), a(3)*sin(th3);
                   0, sin(alpha(3)), cos(alpha(3)), d(3);
                   0, 0, 0, 1];
            
            % Ciclo per variare il giunto 4
            for th4 = theta4_min:passo2:theta4_max
            % Matrice di trasformazione T34 per il giunto 4
                T34 = [cos(th4), -sin(th4)*cos(alpha(4)), sin(th4)*sin(alpha(4)), a(4)*cos(th4);
                       sin(th4), cos(th4)*cos(alpha(4)), -cos(th4)*sin(alpha(4)), a(4)*sin(th4);
                       0, sin(alpha(4)), cos(alpha(4)), d(4);
                       0, 0, 0, 1];
                
                % Ciclo per variare il giunto 5
                for th5 = theta5_min:passo2:theta5_max
                % Matrice di trasformazione T45 per il giunto 5
                    T45 = [cos(th5), -sin(th5)*cos(alpha(5)), sin(th5)*sin(alpha(5)), a(5)*cos(th5);
                           sin(th5), cos(th5)*cos(alpha(5)), -cos(th5)*sin(alpha(5)), a(5)*sin(th5);
                           0, sin(alpha(5)), cos(alpha(5)), d(5);
                           0, 0, 0, 1];

                    % Ciclo per variare il giunto 6
                    for th6 = theta6_min:passo2:theta6_max
                    % Matrice di trasformazione T56 per il giunto 5
                        T56 = [cos(th6), -sin(th6)*cos(alpha(6)), sin(th6)*sin(alpha(6)), a(6)*cos(th6);
                               sin(th6), cos(th6)*cos(alpha(6)), -cos(th6)*sin(alpha(6)), a(6)*sin(th6);
                               0, sin(alpha(6)), cos(alpha(6)), d(6);
                               0, 0, 0, 1];
                    
                        % Matrice di trasformazione totale T06
                        T06 = T01 * T12 * T23 * T34 * T45 * T56;
                        
                        % Posizione dell'end-effector (H) nel sistema di coordinate globale
                        H = T06 * [0; 0; 0; 1];
                        xH(k) = H(1);  % Coordinata X
                        yH(k) = H(2);  % Coordinata Y
                        zH(k) = H(3);  % Coordinata Z
                        k = k + 1;     % Aggiorna il contatore delle posizioni
                    end
                end
            end
        end
    end
end

% Visualizzazione del workspace in 3D
figure(1)
plot3(xH, yH, zH, 'b.');
title('Workspace 3D del Robot ABB IRB 120');
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
grid on;
axis equal;

% Visualizzazione del workspace nel piano XY
figure(2)
plot(xH, yH, 'b.');
title('Workspace XY del Robot ABB IRB 120');
xlabel('X [m]');
ylabel('Y [m]');
grid on;
axis equal;

% Visualizzazione del workspace nel piano XZ
figure(3)
plot(xH, zH, 'b.');
title('Workspace XZ del Robot ABB IRB 120');
xlabel('X [m]');
ylabel('Z [m]');
grid on;
axis equal;

% Visualizzazione del workspace nel piano YZ
figure(4)
plot(yH, zH, 'b.');
title('Workspace YZ del Robot ABB IRB 120');
xlabel('Y [m]');
ylabel('Z [m]');
grid on;
axis equal;
