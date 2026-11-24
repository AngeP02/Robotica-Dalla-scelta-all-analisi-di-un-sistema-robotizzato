%Workspace e Path Planning Robot ABB IRB 120 (RRRRRR)
clear all;
close all;
clc;

%% Parametri di Denavit-Hartenberg per ABB IRB 120
d = [0.29, 0, 0, 0.302, 0, 0.072];  % Offset dei giunti lungo l'asse z (in metri)
a = [0, 0.27, 0.07, 0, 0, 0];  % Lunghezze dei link tra i giunti (in metri)
alpha = [-90, 0, -90, 90, -90, 0] * pi/180;  % Angoli di twist in radianti

% Range di movimento dei giunti
th1_min=-165*pi/180; th1_max=165*pi/180;
th2_min=-110*pi/180; th2_max=110*pi/180;
th3_min=-110*pi/180; th3_max=70*pi/180;
th4_min=-160*pi/180; th4_max=160*pi/180;
th5_min=-120*pi/180; th5_max=120*pi/180;
th6_min=-400*pi/180; th6_max=400*pi/180;

%% Definizione dei punti A e B (posizioni cartesiane) per la traiettoria cubica
A = [0.3, 0, 0.4];  % Punto A [x, y, z] 
B = [0, 0.3, 0.4];   % Punto B [x, y, z]  

% Traiettoria cubica tra i punti A e B
t_vals = linspace(0, 1, 100);  % Tempo normalizzato da 0 a 1

% Condizioni di contorno
th_in = A;            % Posizione iniziale
th_fin = B;           % Posizione finale
th_p_in = [0, 0, 0];  % Velocità iniziale (assunta zero)
th_p_fin = [0, 0, 0]; % Velocità finale (assunta zero)

% Calcolo dei coefficienti del polinomio cubico per ciascuna dimensione
for i = 1:3
    % Matrice delle condizioni di contorno per il polinomio cubico
    A_cubic = [1 t_vals(1) t_vals(1)^2 t_vals(1)^3;
               1 t_vals(end) t_vals(end)^2 t_vals(end)^3;
               0 1 2*t_vals(1) 3*t_vals(1)^2;
               0 1 2*t_vals(end) 3*t_vals(end)^2];
    % Vettore dei valori delle condizioni
    B_cubic = [th_in(i); th_fin(i); th_p_in(i); th_p_fin(i)];
    % Risolvere per i coefficienti del polinomio
    x = inv(A_cubic) * B_cubic;
    a0 = x(1); a1 = x(2); a2 = x(3); a3 = x(4);

    % Definire il polinomio cubico
    syms t
    theta = a0 + a1 * t + a2 * t^2 + a3 * t^3;

    % Calcolare i valori della traiettoria cubica
    path(:, i) = double(subs(theta, t, t_vals));
end

%% Calcolo del workspace
passo1 = pi/9; %passo
passo2 = pi/2; 
k=1;

for th1=th1_min:passo1:th1_max
    
    T01 = [cos(th1), -sin(th1)*cos(alpha(1)), sin(th1)*sin(alpha(1)), a(1)*cos(th1);
        sin(th1), cos(th1)*cos(alpha(1)), -cos(th1)*sin(alpha(1)), a(1)*sin(th1);
        0, sin(alpha(1)), cos(alpha(1)), d(1);
        0, 0 , 0, 1];
    
    for th2=th2_min:passo1:th2_max
        
        T12 = [cos(th2), -sin(th2)*cos(alpha(2)), sin(th2)*sin(alpha(2)), a(2)*cos(th2);
        sin(th2), cos(th2)*cos(alpha(2)), -cos(th2)*sin(alpha(2)), a(2)*sin(th2);
        0, sin(alpha(2)), cos(alpha(2)), d(2);
        0, 0 , 0, 1];
    
            for th3=th3_min:passo1:th3_max

                T23 = [cos(th3), -sin(th3)*cos(alpha(3)), sin(th3)*sin(alpha(3)), a(3)*cos(th3);
                sin(th3), cos(th3)*cos(alpha(3)), -cos(th3)*sin(alpha(3)), a(3)*sin(th3);
                0, sin(alpha(3)), cos(alpha(3)), d(3);
                0, 0 , 0, 1];
            
                for th4=th4_min:passo2:th4_max

                    T34 = [cos(th4), -sin(th4)*cos(alpha(4)), sin(th4)*sin(alpha(4)), a(4)*cos(th4);
                            sin(th4), cos(th4)*cos(alpha(4)), -cos(th4)*sin(alpha(4)), a(4)*sin(th4);
                            0, sin(alpha(4)), cos(alpha(4)), d(4);
                            0, 0 , 0, 1];
                        
                    for th5=th5_min:passo2:th5_max
                
                        T45 = [cos(th5), -sin(th5)*cos(alpha(5)), sin(th5)*sin(alpha(5)), a(5)*cos(th5);
                            sin(th5), cos(th5)*cos(alpha(5)), -cos(th5)*sin(alpha(5)), a(5)*sin(th5);
                            0, sin(alpha(5)), cos(alpha(5)), d(5);
                            0, 0 , 0, 1];
                        
                        for th6=th6_min:passo2:th6_max
                
                        T56 = [cos(th6), -sin(th6)*cos(alpha(6)), sin(th6)*sin(alpha(6)), a(6)*cos(th6);
                            sin(th6), cos(th6)*cos(alpha(6)), -cos(th6)*sin(alpha(6)), a(6)*sin(th6);
                            0, sin(alpha(6)), cos(alpha(6)), d(6);
                            0, 0 , 0, 1];

                        T06=T01*T12*T23*T34*T45*T56;

                        H=T06*[0;0;0;1];
                        xH(k)=H(1);
                        yH(k)=H(2);
                        zH(k)=H(3);

                        k=k+1;
                        end 
                    end 
                end
            end               
     end
end

%% Visualizzazione del workspace e della traiettoria
% Visualizzare il workspace in 3D
figure;
plot3(xH, yH, zH, 'y.');
hold on;

% Visualizzare la traiettoria
plot3(path(:, 1), path(:, 2), path(:, 3), 'b-', 'LineWidth', 4);
xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
title('Workspace e Traiettoria ABB IRB 120');
grid on;
axis equal;
legend('Workspace', 'Traiettoria');
