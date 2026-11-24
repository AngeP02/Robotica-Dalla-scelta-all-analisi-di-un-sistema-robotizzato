% Grafici Path Planning Robot ABB IRB 120 (RRRRRR)
clear all;
close all;
clc;

%% Condizioni iniziali e finali per i giunti   
th_in = [0, 0, 0, 0, pi/2, 0];              %
th_fin = [pi/2, pi/6, -pi/6, 0, pi/2, 0];   %
th_p_in = [0, 0, 0, 0, 0, 0];               % il robot parte fermo, tutte le velcoità sono a 0
th_p_fin = [0, 0, 0, 0, 0, 0];              % il robot arriva fermo, tutte le velocità sono 0
t_ = [0 2];   % Intervallo di tempo [inizio, fine] (realistico per me operaioni da noi descritte)

%% Tipo di giunto: 0 = rotoidale, 1 = prismatico
joint_type = [0, 0, 0, 0, 0, 0];

%% Path Planning per ogni giunto
for i = 1:6
    A = [1 t_(1,1) t_(1,1)^2 t_(1,1)^3;
         1 t_(1,2) t_(1,2)^2 t_(1,2)^3;
         0 1 2*t_(1,1) 3*t_(1,1)^2;
         0 1 2*t_(1,2) 3*t_(1,2)^2];
    B = [th_in(i); th_fin(i); th_p_in(i); th_p_fin(i)];
    x{i} = inv(A) * B;
    a0 = x{i}(1); a1 = x{i}(2); a2 = x{i}(3); a3 = x{i}(4);

    syms t
    theta{i} = a0 + a1 * t + a2 * t^2 + a3 * t^3;
    theta_punto{i} = diff(theta{i}, t, 1);
    theta_due_punti{i} = diff(theta{i}, t, 2);
    theta_tre_punti{i} = diff(theta{i}, t, 3);

    t1 = t_(1,1);
    t2 = t_(1,2);
    figure(i);
    sgtitle(['Giunto ' num2str(i) ' - ABB IRB 120']);
    
    % Grafici
    t1 = t_(1,1);
    t2 = t_(1,2);
    figure(i);
    sgtitle(['Giunto ' num2str(i) ' - ABB IRB 120']);
    
    % Funzione di Spostamento
    subplot(2,2,1);
    ezplot(theta{i}, [t1, t2]);
    title('Funzione di Spostamento');
    xlabel('t [s]');
    if joint_type(i) == 1
        ylabel('Spostamento [mm]');
    else
        ylabel('Spostamento [rad]');
    end
    
    % Funzione di Velocità
    subplot(2,2,2);
    ezplot(theta_punto{i}, [t1, t2]);
    title('Funzione di Velocità');
    xlabel('t [s]');
    if joint_type(i) == 1
        ylabel('Velocità [mm/s]');
    else
        ylabel('Velocità [rad/s]');
    end
    
    % Funzione di Accelerazione
    subplot(2,2,3);
    ezplot(theta_due_punti{i}, [t1, t2]);
    title('Funzione di Accelerazione');
    xlabel('t [s]');
    if joint_type(i) == 1
        ylabel('Accelerazione [mm/s^2]');
    else
        ylabel('Accelerazione [rad/s^2]');
    end
    
    % Funzione di Jerk
    subplot(2,2,4);
    ezplot(theta_tre_punti{i}, [t1, t2]);
    title('Funzione di Jerk');
    xlabel('t [s]');
    if joint_type(i) == 1
        ylabel('Jerk [mm/s^3]');
    else
        ylabel('Jerk [rad/s^3]');
    end
end
