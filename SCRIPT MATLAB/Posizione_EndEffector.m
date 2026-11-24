% definizione dei parametri di Denavit - Hartemberg
d = [0.29 0 0 0.302 0 0.072];       
a = [0 0.27 0.07 0 0 0];
alpha = [-pi/2 0 -pi/2 pi/2 -pi/2 0];

theta = [0 -pi /2 0 0 0 0];

% definizione delle matrici di trasformazione
T01 = [cos(theta(1)), -sin(theta(1))*cos(alpha(1)), sin(theta(1))*sin(alpha(1)), a(1)*cos(theta(1));
       sin(theta(1)), cos(theta(1))*cos(alpha(1)), -cos(theta(1))*sin(alpha(1)), a(1)*sin(theta(1));
       0, sin(alpha(1)), cos(alpha(1)), d(1);
       0, 0, 0, 1];

T12 = [cos(theta(2)), -sin(theta(2))*cos(alpha(2)), sin(theta(2))*sin(alpha(2)), a(2)*cos(theta(2));
       sin(theta(2)), cos(theta(2))*cos(alpha(2)), -cos(theta(2))*sin(alpha(2)), a(2)*sin(theta(2));
       0, sin(alpha(2)), cos(alpha(2)), d(2);
       0, 0, 0, 1];

T23 = [cos(theta(3)), -sin(theta(3))*cos(alpha(3)), sin(theta(3))*sin(alpha(3)), a(3)*cos(theta(3));
       sin(theta(3)), cos(theta(3))*cos(alpha(3)), -cos(theta(3))*sin(alpha(3)), a(3)*sin(theta(3));
       0, sin(alpha(3)), cos(alpha(3)), d(3);
       0, 0, 0, 1];

T34 = [cos(theta(4)), -sin(theta(4))*cos(alpha(4)), sin(theta(4))*sin(alpha(4)), a(4)*cos(theta(4));
       sin(theta(4)), cos(theta(4))*cos(alpha(4)), -cos(theta(4))*sin(alpha(4)), a(4)*sin(theta(4));
       0, sin(alpha(4)), cos(alpha(4)), d(4);
       0, 0, 0, 1];

T45 = [cos(theta(5)), -sin(theta(5))*cos(alpha(5)), sin(theta(5))*sin(alpha(5)), a(5)*cos(theta(5));
       sin(theta(5)), cos(theta(5))*cos(alpha(5)), -cos(theta(5))*sin(alpha(5)), a(5)*sin(theta(5));
       0, sin(alpha(5)), cos(alpha(5)), d(5);
       0, 0, 0, 1];

T56 = [cos(theta(6)), -sin(theta(6))*cos(alpha(6)), sin(theta(6))*sin(alpha(6)), a(6)*cos(theta(6));
       sin(theta(6)), cos(theta(6))*cos(alpha(6)), -cos(theta(6))*sin(alpha(6)), a(6)*sin(theta(6));
       0, sin(alpha(6)), cos(alpha(6)), d(6);
       0, 0, 0, 1];

T06 = T01*T12*T23*T34*T45*T56;

% stampa del risultato
pos = T06(1:3, 4);
disp("Vettore posizione dell’end-effector:");
disp(pos);

 