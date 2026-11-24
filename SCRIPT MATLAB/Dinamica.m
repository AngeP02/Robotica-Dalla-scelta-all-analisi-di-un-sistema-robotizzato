% Analisi Dinamica Robot ABB IRB 120 (6 DOF) con metodo Ricorsivo di
% Newton-Euler

clear all
close all
clc

%impostazione del sistema lineare
th_in = [0, 0, 0, 0, pi/2, 0];   
th_fin = [pi/2, pi/6, -pi/6, 0, pi/2, 0];

%VELOCITA' INIZIALI dei motori
th_p_in = [0 0 0 0 0 0];
th_p_fin = [0 0 0 0 0 0];

t_ = [0 1]; %intervallo di tempo espresso in secondi
tstep = 0.02; %step per il calcolo dei valori temporali che vanno da t=0s e t=1s (scelto a partire dalla frequenza di clock del controllore, 20 ms)
ncicli = (t_(2)/tstep)+1; %numero di cicli

%Per eliminare le vibrazioni del robot (e ridurne l'usura) uso un polinomio interpolante del terzo ordine theta(t)=a0+a1*t+a2*t^2+a3*t3
for i=1:1:6
    %matrice dei coefficienti
    A = [1  t_(1,1)  t_(1,1)^2  t_(1,1)^3;
         1  t_(1,2)  t_(1,2)^2  t_(1,2)^3;
         0  1        2*t_(1,1)  3*t_(1,1)^2;
         0  1        2*t_(1,2)  3*t_(1,2)^2];

    %vettore dei termini noti
    B = [th_in(1,i); th_fin(1,i); th_p_in(1,i); th_p_fin(1,i)];	

    a=inv(A);	%matrice inversa di A

    x{i}=a*B;	%vettore soluzione

    a0(i)=x{i}(1);
    a1(i)=x{i}(2);
    a2(i)=x{i}(3);
    a3(i)=x{i}(4);
end

%sistema di riferimento H, solidale all'elemento terminale (end effector)
alpha_h=0; %[deg]
a_h=0; %[m]
d_h=0; %[m]

%Parametri Denavit&Hartemberg
d = [0.29 0 0 0.302 0 0.072];          %[m] LINK OFFSET, distanza tra x_i e x_i+1 misurata lungo z_i+1      
a = [0 0.27 0.07 0 0 0];               %[m] LINK LENGHT, distanza tra l'asse z_i e z_i+1 misurata lungo x_i
alpha = [-pi/2 0 -pi/2 pi/2 -pi/2 0];  %[deg] TWIST ANGLE, angolo tra l'asse z_i e z_i+1 ruotando intorno a x_i

%Funzioni spostamento , velocità ed accelerazione dei motori
c = 0;
for n = 1:6
    for t = t_(1):tstep:t_(2)
        c = c+1;
        theta = a0(n) + a1(n)*t + a2(n)*t^2 + a3(n)*t^3;
        theta_punto = a1(n) + 2*a2(n)*t + 3*a3(n)*t^2;
        theta_2_punti = 2*a2(n) + 6*a3(n)*t;
        
        th(c,n) = theta; %spostamento
        th_p(c,n) = theta_punto; %velocità
        th_2p(c,n) = theta_2_punti; %accelerazione
    end
end

%Matrici di trasformazione Ti-1,i
for j=1:6
    for i=1:ncicli
        T{i,j}=  [       cos(th(i,j))                     -sin(th(i,j))                      0                      a(j)
                   sin(th(i,j))*cos(alpha(j))       cos(th(i,j))*cos(alpha(j))        -sin(alpha(j))          -sin(alpha(j))*d(j)
                   sin(th(i,j))*sin(alpha(j))       cos(th(i,j))*sin(alpha(j))         cos(alpha(j))           cos(alpha(j))*d(j)
                              0                                 0                            0                        1             ];
    end
end


%Estrapolazione della matrice di Rotazione Ri,i+1 da quella di Trasformazione Ti-1,i (DA USARE PER L'EQUAZIONI DI EQUILIBRIO)
for i=1:1:ncicli
    R{i,1}=T{i,1}(1:3,1:3); %R01
    R{i,2}=T{i,2}(1:3,1:3); %R12
    R{i,3}=T{i,3}(1:3,1:3); %R23
    R{i,4}=T{i,4}(1:3,1:3); %R34
    R{i,5}=T{i,5}(1:3,1:3); %R45
    R{i,6}=T{i,6}(1:3,1:3); %R56
end

%Matrice di Rotazione Ri+1,i(DA USARE PER L'EQUAZIONI DI VELOCITA' ED ACCELERAZIONE), ne eseguo la trasposta
for i=1:1:ncicli
    Rt{i,1}=R{i,1}'; %R10
    Rt{i,2}=R{i,2}'; %R21
    Rt{i,3}=R{i,3}'; %R32
    Rt{i,4}=R{i,4}'; %R43
    Rt{i,5}=R{i,5}'; %R54
    Rt{i,6}=R{i,6}'; %R65
end

%Estrapolazione dei vettori posizione (dei motori) dalla matrice di Trasformazione
for i=1:1:ncicli
    P{i,1}=T{i,1}(1:3,4); %P01 [m]
    P{i,2}=T{i,2}(1:3,4); %P12 [m]
    P{i,3}=T{i,3}(1:3,4); %P23 [m]
    P{i,4}=T{i,4}(1:3,4); %P34 [m]
    P{i,5}=T{i,5}(1:3,4); %P45 [m]
    P{i,6}=T{i,6}(1:3,4); %P56 [m]
end

%Vettori posizione del Baricentro usando come sistema di riferimento quello baricentrico
PG = cell(1,6);
PG{1} = [0; -0.02; 0.1];   % Baricentro Link 1 (Base rotante): spostato leggermente lungo Z
PG{2} = [0.13; 0; 0.05];   % Baricentro Link 2 (Braccio inferiore): a circa metà lunghezza (0.27m)
PG{3} = [0.03; 0; 0.05];   % Baricentro Link 3 (Gomito/Braccio superiore)
PG{4} = [0.05; 0; 0];      % Baricentro Link 4 (Avambraccio rotante)
PG{5} = [0.02; 0; 0];      % Baricentro Link 5 (Polso)
PG{6} = [0; 0; 0.01];      % Baricentro Link 6 (Flangia): praticamente sull'asse


% --> ANALISI CINEMATICA
g = [0;0;-9.807]; %accelerazione di gravità
v_ang_in = [0;0;0]; %velocità angolare iniziale (ne scrivo 3, così ho tutte matrici con numero di righe uniformi a Rt, P, PG, th, th_p, th_2p)
a_lin_in = [0;0;0]; %accelerazione lineare iniziale (ne scrivo 3, così ho tutte matrici con numero di righe uniformi a Rt, P, PG, th, th_p, th_2p)
a_ang_in = [0;0;0]; %accelerazione angolare iniziale (ne scrivo 3, così ho tutte matrici con numero di righe uniformi a Rt, P, PG, th, th_p, th_2p)
Z = [0;0;1]; %Versore lungo Z

for j=1:1:6  %outward iteration
    for i=1:1:ncicli
        if j==1
            v_ang{i,j} = Rt{i,j}*v_ang_in+th_p(i,j)*Z; %velocità angolare OMEGA
            a_ang{i,j} = Rt{i,j}*a_ang_in+th_2p(i,j)*Z+cross(Rt{i,j}*v_ang_in,th_p(i,j)*Z); %accelerazione angolare OMEGA_PUNTO 
            a_lin{i,j} = Rt{i,j}*(cross(a_ang_in,P{i,j})+cross(v_ang_in,cross(v_ang_in,P{i,j}))+a_lin_in); %accelerazione lineare V_PUNTO
            a_bar{i,j} = cross(a_ang_in,PG{j})+cross(v_ang_in,cross(v_ang_in,PG{j}))+a_lin_in+g; %accelerazione nel baricentro V_PUNTO_G
        else
            v_ang{i,j} = Rt{i,j}*v_ang{i,j-1}+th_p(i,j)*Z; %velocità angolare OMEGA
            a_ang{i,j} = Rt{i,j}*a_ang{i,j-1}+th_2p(i,j)*Z+cross(Rt{i,j}*v_ang{i,j-1},th_p(i,j)*Z);%accelerazione angolare OMEGA_PUNTO
            a_lin{i,j} = Rt{i,j}*(cross(a_ang{i,j-1},P{i,j})+cross(v_ang{i,j-1},cross(v_ang{i,j-1},P{i,j}))+a_lin{i,j-1}); %accelerazione lineare V_PUNTO
            a_bar{i,j} = cross(a_ang{i,j-1},PG{j})+cross(v_ang{i,j-1},cross(v_ang{i,j-1},PG{j}))+a_lin{i,j-1}; %accelerazione nel baricentro V_PUNTO_G
        end
    end
end


% --> DETERMINAZIONE DELLE FORZE E MOMENTI D'INERZIA
%Matrici dei momenti d'inerzia [kg*m^2]
I = cell(1,6);
% Link 1
I{1} = [0.014   0       0; 
        0       0.014   0; 
        0       0       0.010]; 
% Link 2
I{2} = [0.06   0       0; 
        0       0.041   0; 
        0       0       0.025]; 
% Link 3
I{3} = [0.008   0       0; 
        0       0.016   0; 
        0       0       0.012]; 
% Link 4
I{4} = [0.002   0       0; 
        0       0.004   0; 
        0       0       0.005]; 
% Link 5
I{5} = [0.0004  0       0; 
        0       0.0008  0; 
        0       0       0.0008]; 
% Link 6
I{6} = [0.001  0       0; 
        0       0.001  0; 
        0       0       0.001];

  
%Masse dei motori + link
M = [3.06, 3.90, 2.94, 1.32, 0.54, 0.13]; %[Kg]

for j=1:1:6
    for i=1:1:ncicli
        F_in{i,j}=-M(1,j)*a_bar{i,j}; %Forze d'inerzia
        N_in{i,j}=I{j}*a_ang{i,j}+cross(v_ang{i,j},(I{j}*v_ang{i,j})); %Momenti d'inerzia
    end
end


% --> EQUAZIONI DI EQUILIBRIO
F_est=[0;0;2.943]; %P = m * g = 0.3kg * 9.81 m/s^2 = 2.94 N.
N_est=[0;0;0];

for j=6:-1:1 %inward iteration
    for i=1:1:ncicli
        if j==6
            F{i,j} = R{i,j}*F_est+F_in{i,j}; %Equazione di equilibrio delle forze
            N{i,j} = R{i,j}*N_est+cross(P{i,j},(R{i,j}*F_est))+N_in{i,j}+cross(PG{j},F_in{i,j}); %Equazione di equilibrio dei momenti
        else
            F{i,j} = R{i,j}*F{i,j+1}+F_in{i,j}; %Equazione di equilibrio delle forze
            N{i,j} = R{i,j}*N{i,j+1}+cross(P{i,j},(R{i,j}*F{i,j+1}))+N_in{i,j}+cross(PG{j},F_in{i,j}); %Equazione di equilibrio dei momenti
        end
    end
end


%%%%%%%%%%%%%%


% --> PLOT dei Risultati
k=0;
for i=t_(1):tstep:t_(2)
    k=k+1;
    tempo(k)=i;
end

%MODULI DEI VETTORI
for j=1:1:6
    for i=1:1:ncicli
        A_LIN(i,j)=sqrt(a_lin{i,j}(1)^2+a_lin{i,j}(2)^2+a_lin{i,j}(3)^2); %Accelerazione lineare
        A_BAR(i,j)=sqrt(a_bar{i,j}(1)^2+a_bar{i,j}(2)^2+a_bar{i,j}(3)^2); %Accelerazione baricentro
        F_IN(i,j)=sqrt(F_in{i,j}(1)^2+F_in{i,j}(2)^2+F_in{i,j}(3)^2); %Forze d'inerzia
        N_IN(i,j)=sqrt(N_in{i,j}(1)^2+N_in{i,j}(2)^2+N_in{i,j}(3)^2); %Momento d'inerzia
        F_EQ(i,j)=sqrt(F{i,j}(1)^2+F{i,j}(2)^2+F{i,j}(3)^2); %Equilibrio delle forze
        N_EQ(i,j)=sqrt(N{i,j}(1)^2+N{i,j}(2)^2+N{i,j}(3)^2); %Equilibrio dei momenti
    end
end

%GRAFICI VELOCITA' ANGOLARI
for i=1:1:ncicli
    v_angolare1(i)=v_ang{i,1}(3,1);
    v_angolare2(i)=v_ang{i,2}(3,1);
    v_angolare3(i)=v_ang{i,3}(3,1);
    v_angolare4(i)=v_ang{i,4}(3,1);
    v_angolare5(i)=v_ang{i,5}(3,1);
    v_angolare6(i)=v_ang{i,6}(3,1);
end

figure(1);
subplot(2,3,1)
plot(tempo,v_angolare1); %velocità angolare 1
grid on
title('Velocità angolare LINK 1')
xlabel('t [s]')
ylabel('Velocità angolare [rad/s]')

subplot(2,3,2);
plot(tempo,v_angolare2) %velocità angolare 2
grid on
title('Velocità angolare LINK 2')
xlabel('t [s]')
ylabel('Velocità angolare [rad/s]')

subplot(2,3,3);
plot(tempo,v_angolare3) %velocità angolare 3
grid on
title('Velocità angolare LINK 3')
xlabel('t [s]')
ylabel('Velocità angolare [rad/s]')

subplot(2,3,4);
plot(tempo,v_angolare4) %velocità angolare 4
grid on
title('Velocità angolare LINK 4')
xlabel('t [s]')
ylabel('Velocità angolare [rad/s]')

subplot(2,3,5);
plot(tempo,v_angolare5) %velocità angolare 5
grid on
title('Velocità angolare LINK 5')
xlabel('t [s]')
ylabel('Velocità angolare [rad/s]')

subplot(2,3,6);
plot(tempo,v_angolare6) %velocità angolare 6
grid on
title('Velocità angolare LINK 6')
xlabel('t [s]')
ylabel('Velocità angolare [rad/s]')

%GRAFICI ACCELERAZIONI ANGOLARI
for i=1:1:ncicli
    a_angolare1(i)=a_ang{i,1}(3,1);
    a_angolare2(i)=a_ang{i,2}(3,1);
    a_angolare3(i)=a_ang{i,3}(3,1);
    a_angolare4(i)=a_ang{i,4}(3,1);
    a_angolare5(i)=a_ang{i,5}(3,1);
    a_angolare6(i)=a_ang{i,6}(3,1);
end

figure(2);
subplot(2,3,1);
plot(tempo,a_angolare1) %accelerazione angolare 1
grid on
title('Accelerazione angolare LINK 1')
xlabel('t [s]')
ylabel('Accelerazione angolare [rad/s^2]')

subplot(2,3,2);
plot(tempo,a_angolare2) %accelerazione angolare 2
grid on
title('Accelerazione angolare LINK 2')
xlabel('t [s]')
ylabel('Accelerazione angolare [rad/s^2]')

subplot(2,3,3);
plot(tempo,a_angolare3) %accelerazione angolare 3
grid on
title('Accelerazione angolare LINK 3')
xlabel('t [s]')
ylabel('Accelerazione angolare [rad/s^2]')

subplot(2,3,4);
plot(tempo,a_angolare4) %accelerazione angolare 4
grid on
title('Accelerazione angolare LINK 4')
xlabel('t [s]')
ylabel('Accelerazione angolare [rad/s^2]')

subplot(2,3,5);
plot(tempo,a_angolare5) %accelerazione angolare 5
grid on
title('Accelerazione angolare LINK 5')
xlabel('t [s]')
ylabel('Accelerazione angolare [rad/s^2]')

subplot(2,3,6);
plot(tempo,a_angolare6) %accelerazione angolare 6
grid on
title('Accelerazione angolare LINK 6')
xlabel('t [s]')
ylabel('Accelerazione angolare [rad/s^2]')

%GRAFICI ACCELERAZIONI LINEARI
for i=1:1:ncicli
    a_lineare1(i)=A_LIN(i,1);
    a_lineare2(i)=A_LIN(i,2);
    a_lineare3(i)=A_LIN(i,3);
    a_lineare4(i)=A_LIN(i,4);
    a_lineare5(i)=A_LIN(i,5);
    a_lineare6(i)=A_LIN(i,6);
end

figure(3);
subplot(2,3,1);
plot(tempo,a_lineare1) %accelerazione lineare 1
grid on
title('Accelerazione lineare LINK 1')
xlabel('t [s]')
ylabel('Accelerazione lineare [m/s^2]')

subplot(2,3,2);
plot(tempo,a_lineare2) %accelerazione lineare 2
grid on
title('Accelerazione lineare LINK 2')
xlabel('t [s]')
ylabel('Accelerazione lineare [m/s^2]')

subplot(2,3,3);
plot(tempo,a_lineare3) %accelerazione lineare 3
grid on
title('Accelerazione lineare LINK 3')
xlabel('t [s]')
ylabel('Accelerazione lineare [m/s^2]')

subplot(2,3,4);
plot(tempo,a_lineare4) %accelerazione lineare 4
grid on
title('Accelerazione lineare LINK 4')
xlabel('t [s]')
ylabel('Accelerazione lineare [m/s^2]')

subplot(2,3,5);
plot(tempo,a_lineare5) %accelerazione lineare 5
grid on
title('Accelerazione lineare LINK 5')
xlabel('t [s]')
ylabel('Accelerazione lineare [m/s^2]')

subplot(2,3,6);
plot(tempo,a_lineare6) %accelerazione lineare 6
grid on
title('Accelerazione lineare LINK 6')
xlabel('t [s]')
ylabel('Accelerazione lineare [m/s^2]')


%GRAFICI ACCELERAZIONI NEL BARICENTRO
for i=1:1:ncicli
    a_baricentro1(i)=A_BAR(i,1);
    a_baricentro2(i)=A_BAR(i,2);
    a_baricentro3(i)=A_BAR(i,3);
    a_baricentro4(i)=A_BAR(i,4);
    a_baricentro5(i)=A_BAR(i,5);
    a_baricentro6(i)=A_BAR(i,6);
end

figure(4);
subplot(2,3,1);
plot(tempo,a_baricentro1) %accelerazione baricentro 1
grid on
title('Accelerazione baricentro LINK 1')
xlabel('t [s]')
ylabel('Accelerazione baricentro [m/s^2]')

subplot(2,3,2);
plot(tempo,a_baricentro2) %accelerazione baricentro 2
grid on
title('Accelerazione baricentro LINK 2')
xlabel('t [s]')
ylabel('Accelerazione baricentro [m/s^2]')

subplot(2,3,3);
plot(tempo,a_baricentro3) %accelerazione baricentro 3
grid on
title('Accelerazione baricentro LINK 3')
xlabel('t [s]')
ylabel('Accelerazione baricentro [m/s^2]')

subplot(2,3,4);
plot(tempo,a_baricentro4) %accelerazione baricentro 4
grid on
title('Accelerazione baricentro LINK 4')
xlabel('t [s]')
ylabel('Accelerazione baricentro [m/s^2]')

subplot(2,3,5);
plot(tempo,a_baricentro5) %accelerazione baricentro 5
grid on
title('Accelerazione baricentro LINK 5')
xlabel('t [s]')
ylabel('Accelerazione baricentro [m/s^2]')

subplot(2,3,6);
plot(tempo,a_baricentro6) %accelerazione baricentro 6
grid on
title('Accelerazione baricentro LINK 6')
xlabel('t [s]')
ylabel('Accelerazione baricentro [m/s^2]')


%GRAFICI FORZE E MOMENTI

%FORZE D'INERZIA
for i=1:1:ncicli
    f_inerzia1(i)=F_IN(i,1);
    f_inerzia2(i)=F_IN(i,2);
    f_inerzia3(i)=F_IN(i,3);
    f_inerzia4(i)=F_IN(i,4);
    f_inerzia5(i)=F_IN(i,5);
    f_inerzia6(i)=F_IN(i,6);
end

figure(5);
subplot(2,3,1);
plot(tempo,f_inerzia1) %forza d'inerzia 1
grid on
title('Forza inerzia LINK 1')
xlabel('t [s]')
ylabel('Forza inerzia [N]')

subplot(2,3,2);
plot(tempo,f_inerzia2) %forza d'inerzia 2
grid on
title('Forza inerzia LINK 2')
xlabel('t [s]')
ylabel('Forza inerzia [N]')

subplot(2,3,3);
plot(tempo,f_inerzia3) %forza d'inerzia 3
grid on
title('Forza inerzia LINK 3')
xlabel('t [s]')
ylabel('Forza inerzia [N]')

subplot(2,3,4);
plot(tempo,f_inerzia4) %forza d'inerzia 4
grid on
title('Forza inerzia LINK 4')
xlabel('t [s]')
ylabel('Forza inerzia [N]')

subplot(2,3,5);
plot(tempo,f_inerzia5) %forza d'inerzia 5
grid on
title('Forza inerzia LINK 5')
xlabel('t [s]')
ylabel('Forza inerzia [N]')

subplot(2,3,6);
plot(tempo,f_inerzia6) %forza d'inerzia 6
grid on
title('Forza inerzia LINK 6')
xlabel('t [s]')
ylabel('Forza inerzia [N]')

%MOMENTI D'INERZIA
for i=1:1:ncicli
    n_inerzia1(i)=N_IN(i,1);
    n_inerzia2(i)=N_IN(i,2);
    n_inerzia3(i)=N_IN(i,3);
    n_inerzia4(i)=N_IN(i,4);
    n_inerzia5(i)=N_IN(i,5);
    n_inerzia6(i)=N_IN(i,6);
end

figure(6);
subplot(2,3,1);
plot(tempo,n_inerzia1) %momento d'inerzia 1
grid on
title('Momento inerzia LINK 1')
xlabel('t [s]')
ylabel('Momento inerzia [Kg*m^2]')

subplot(2,3,2);
plot(tempo,n_inerzia2) %momento d'inerzia 2
grid on
title('Momento inerzia LINK 2')
xlabel('t [s]')
ylabel('Momento inerzia [Kg*m^2]')

subplot(2,3,3);
plot(tempo,n_inerzia3) %momento d'inerzia 3
grid on
title('Momento inerzia LINK 3')
xlabel('t [s]')
ylabel('Momento inerzia [Kg*m^2]')

subplot(2,3,4);
plot(tempo,n_inerzia4) %momento d'inerzia 4
grid on
title('Momento inerzia LINK 4')
xlabel('t [s]')
ylabel('Momento inerzia [Kg*m^2]')

subplot(2,3,5);
plot(tempo,n_inerzia5) %momento d'inerzia 5
grid on
title('Momento inerzia LINK 5')
xlabel('t [s]')
ylabel('Momento inerzia [Kg*m^2]')

subplot(2,3,6);
plot(tempo,n_inerzia6) %momento d'inerzia 6
grid on
title('Momento inerzia LINK 6')
xlabel('t [s]')
ylabel('Momento inerzia [Kg*m^2]')


%EQUILIBRIO DELLE FORZE
for i=1:1:ncicli
    f_eq1(i)=F_EQ(i,1);
    f_eq2(i)=F_EQ(i,2);
    f_eq3(i)=F_EQ(i,3);
    f_eq4(i)=F_EQ(i,4);
    f_eq5(i)=F_EQ(i,5);
    f_eq6(i)=F_EQ(i,6);
end

figure(7);
subplot(2,3,1);
plot(tempo,f_eq1) %forza 1
grid on
title('Forza LINK 1')
xlabel('t [s]')
ylabel('Forza [N]')

subplot(2,3,2);
plot(tempo,f_eq2) %forza 2
grid on
title('Forza LINK 2')
xlabel('t [s]')
ylabel('Forza [N]')

subplot(2,3,3);
plot(tempo,f_eq3) %forza 3
grid on
title('Forza LINK 3')
xlabel('t [s]')
ylabel('Forza [N]')

subplot(2,3,4);
plot(tempo,f_eq4) %forza 4
grid on
title('Forza LINK 4')
xlabel('t [s]')
ylabel('Forza [N]')

subplot(2,3,5);
plot(tempo,f_eq5) %forza 5
grid on
title('Forza LINK 5')
xlabel('t [s]')
ylabel('Forza [N]')

subplot(2,3,6);
plot(tempo,f_eq6) %forza 6
grid on
title('Forza LINK 6')
xlabel('t [s]')
ylabel('Forza [N]')


%EQUILIBRIO DEI MOMENTI
for i=1:1:ncicli
    n_eq1(i)=N_EQ(i,1);
    n_eq2(i)=N_EQ(i,2);
    n_eq3(i)=N_EQ(i,3);
    n_eq4(i)=N_EQ(i,4);
    n_eq5(i)=N_EQ(i,5);
    n_eq6(i)=N_EQ(i,6);
end

figure(8);
subplot(2,3,1);
plot(tempo,n_eq1) %momento di equilibrio 1
grid on
title('Momento LINK 1')
xlabel('t [s]')
ylabel('Momento [N*m]')

subplot(2,3,2);
plot(tempo,n_eq2) %momento di equilibrio 2
grid on
title('Momento LINK 2')
xlabel('t [s]')
ylabel('Momento [N*m]')

subplot(2,3,3);
plot(tempo,n_eq3) %momento di equilibrio 3
grid on
title('Momento LINK 3')
xlabel('t [s]')
ylabel('Momento [N*m]')

subplot(2,3,4);
plot(tempo,n_eq4) %momento di equilibrio 4
grid on
title('Momento LINK 4')
xlabel('t [s]')
ylabel('Momento [N*m]')

subplot(2,3,5);
plot(tempo,n_eq5) %momento di equilibrio 5
grid on
title('Momento LINK 5')
xlabel('t [s]')
ylabel('Momento [N*m]')

subplot(2,3,6);
plot(tempo,n_eq6) %momento di equilibrio 6
grid on
title('Momento LINK 6')
xlabel('t [s]')
ylabel('Momento [N*m]')

%COPPIE DEI MOTORI
for i=1:1:ncicli
    tau1(i,1)=N{i,1}(3,1); %Coppie link 1
    tau2(i,1)=N{i,2}(3,1); %Coppie link 2
    tau3(i,1)=N{i,3}(3,1); %Coppie link 3
    tau4(i,1)=N{i,4}(3,1); %Coppie link 4
    tau5(i,1)=N{i,5}(3,1); %Coppie link 5
    tau6(i,1)=N{i,6}(3,1); %Coppie link 6
end

figure(9);
subplot(2,3,1);
plot(tempo,tau1) %coppia motore 1
grid on
title('Coppia Motore 1')
xlabel('t [s]')
ylabel('Coppia [N*m]')

subplot(2,3,2);
plot(tempo,tau2) %coppia motore 2
grid on
title('Coppia Motore 2')
xlabel('t [s]')
ylabel('Coppia [N*m]')

subplot(2,3,3);
plot(tempo,tau3) %coppia motore 3
grid on
title('Coppia Motore 3')
xlabel('t [s]')
ylabel('Coppia [N*m]')

subplot(2,3,4);
plot(tempo,tau4) %coppia motore 4
grid on
title('Coppia Motore 4')
xlabel('t [s]')
ylabel('Coppia [N*m]')

subplot(2,3,5);
plot(tempo,tau5) %coppia motore 5
grid on
title('Coppia Motore 5')
xlabel('t [s]')
ylabel('Coppia [N*m]')

subplot(2,3,6);
plot(tempo,tau6) %coppia motore 6
grid on
title('Coppia Motore 6')
xlabel('t [s]')
ylabel('Coppia [N*m]')