clear
close all
clc

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% TIEMPO %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

tf = 13.9;           % Tiempo de simulacion en segundos (s)
ts = 0.1;            % Tiempo de muestreo en segundos (s)
t = 0: ts: tf;       % Vector de tiempo
N = length(t);       % Muestras

%%%%%%%%%%%%%%%%%%%%%%%% CONDICIONES INICIALES %%%%%%%%%%%%%%%%%%%%%%%%%%%%

x1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje x) en metros (m)
y1 = zeros (1,N+1);  % Posición en el centro del eje que une las ruedas (eje y) en metros (m)
phi = zeros(1, N+1); % Orientacion del robot en radianes (rad)

x1(1) = 0;    % Posicion inicial eje x
y1(1) = 6;   % Posicion inicial eje y
phi(1) = 0;   % Orientacion inicial del robot

%%%%%%%%%%%%%%%%%%%%%%%%%%%% PUNTO DE CONTROL %%%%%%%%%%%%%%%%%%%%%%%%%%%%

hx = zeros(1, N+1);  % Posicion en el punto de control (eje x) en metros (m) 
hy = zeros(1, N+1);  % Posicion en el punto de control (eje y) en metros (m)

hx(1) = x1(1); % Posicion en el punto de control del robot en el eje x
hy(1) = y1(1); % Posicion en el punto de control del robot en el eje y


%%%%%%%%%%%%%%%%%%%%%% VELOCIDADES DE REFERENCIA %%%%%%%%%%%%%%%%%%%%%%%%%%

%{
% Velocidades para figura 1: colocar 23.9 en tf
u = zeros(1,N); % Inicializamos vector de velocidad lineal
w = zeros(1,N); % Inicializamos vector de velocidad angular
% Definición de tramos para el cuadrado
% Tramo 1
u(1:50) = 1;
% Giro 1
w(51:60) = -(pi/2);
% Tramo 2
u(61:110) = 1;
% Giro 2
w(111:120) = -(pi/2);
% Tramo 3
u(121:170) = 1;
% Giro 3
w(171:180) = -(pi/2);
% Tramo 4
u(181:230) = 1;
% Giro 4
w(231:240) = -(pi/2);
%}

%{
% Velocidades para figura 2: colocar 6.9 en tf
u = zeros(1,N); w = zeros(1,N); 
u(1:10) = 1;                    % Tramo 1
w(11:20) = ((120*pi)/180);      % Giro 1
u(21:30) = 1;                   % Tramo 2
w(31:40) = -(13*pi/18);         % Giro 2
u(41:50) = 1.5;                 % Tramo 3
w(51:60) = -(pi/2);             % Giro 3
u(61:70) = 2.2;                 % Tramo 4
%}

% Velocidades para figura 3: colocar 13.9 en tf
u = [ones(1,20) zeros(1,10) ones(1,15) zeros(1,10) ones(1,20) zeros(1,10) ones(1,20) zeros(1,10) ones(1,25)]; % Velocidad lineal de referencia (m/s)
w = [zeros(1,20) -pi/2*ones(1,10) zeros(1,15) -pi/2*ones(1,10) zeros(1,20) pi/2*ones(1,10) zeros(1,20) pi/2*ones(1,10) zeros(1,25)]; % Velocidad angular de referencia (rad/s)

%%%%%%%%%%%%%%%%%%%%%%%%% BUCLE DE SIMULACION %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k=1:N 
    
    phi(k+1)=phi(k)+w(k)*ts; % Integral numérica (método de Euler)
    
    %%%%%%%%%%%%%%%%%%%%% MODELO CINEMATICO %%%%%%%%%%%%%%%%%%%%%%%%%
    %Aplicamos el modelo cinemático diferencial para obtener las
    %velocidades en x, y, phi
    xp1=u(k)*cos(phi(k+1)); 
    yp1=u(k)*sin(phi(k+1));
    phip= w(k);

    x1(k+1)=x1(k) + xp1*ts ; % Integral numérica (método de Euler)
    y1(k+1)=y1(k) + yp1*ts ; % Integral numérica (método de Euler)
    
    
    % Posicion del robot con respecto al punto de control
    hx(k+1)=x1(k+1); 
    hy(k+1)=y1(k+1);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SIMULACION VIRTUAL 3D %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% a) Configuracion de escena

scene=figure;  % Crear figura (Escena)
set(scene,'Color','white'); % Color del fondo de la escena
set(gca,'FontWeight','bold') ;% Negrilla en los ejes y etiquetas
sizeScreen=get(0,'ScreenSize'); % Retorna el tamaño de la pantalla del computador
set(scene,'position',sizeScreen); % Congigurar tamaño de la figura
camlight('headlight'); % Luz para la escena
axis equal; % Establece la relación de aspecto para que las unidades de datos sean las mismas en todas las direcciones.
grid on; % Mostrar líneas de cuadrícula en los ejes
box on; % Mostrar contorno de ejes
xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)'); % Etiqueta de los eje

view([25 25]); % Orientacion de la figura
axis([-3 10 -3 10 0 2]); % Ingresar limites minimos y maximos en los ejes x y z [minX maxX minY maxY minZ maxZ]

% b) Graficar robots en la posicion inicial
scale = 4;
MobileRobot_5;
H1=MobilePlot_4(x1(1),y1(1),phi(1),scale);hold on;

% c) Graficar Trayectorias
H2=plot3(hx(1),hy(1),0,'r','lineWidth',2);

% d) Bucle de simulacion de movimiento del robot

step=1; % pasos para simulacion

for k=1:step:N

    delete(H1);    
    delete(H2);
    
    H1=MobilePlot_4(x1(k),y1(k),phi(k),scale);
    H2=plot3(hx(1:k),hy(1:k),zeros(1,k),'r','lineWidth',2);
    
    pause(ts);

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%% GRAFICAS %%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Gráficas de velocidades de referencia
figure;
subplot(211)
plot(t,u,'b','LineWidth',2)
grid on
xlabel('Tiempo [s]')
ylabel('u [m/s]')
title('Velocidad lineal')
legend('u')

subplot(212)
plot(t,w,'r','LineWidth',2)
grid on
xlabel('Tiempo [s]')
ylabel('\omega [rad/s]')
title('Velocidad angular')
legend('\omega')


% Gráficas de la pose
figure;
subplot(311)
plot(t, x1(1:N), 'g', 'LineWidth', 2)
grid on
xlabel('Tiempo [s]')
ylabel('x [m]')
title('Posición en x')
legend('x')

subplot(312)
plot(t, y1(1:N), 'm', 'LineWidth', 2)
grid on
xlabel('Tiempo [s]')
ylabel('y [m]')
title('Posición en y')
legend('y')

subplot(313)
plot(t, phi(1:N), 'LineWidth', 2, 'Color', [1 0.5 0])
grid on
xlabel('Tiempo [s]')
ylabel('\phi [rad]')
title('Orientación')
legend('\phi')


%%%%%%%%%%%%%%%%%%%% VELOCIDADES ANGULARES DE RUEDAS %%%%%%%%%%%%%%%%%%%%

r = 0.05;   % radio de rueda
L = 0.17;   % distancia entre ruedas

wl = (u - (w*L)/2)/r;
wr = (u + (w*L)/2)/r;

figure;
subplot(211)
plot(t, wl, 'b', 'LineWidth', 2)
grid on
xlabel('Tiempo [s]')
ylabel('\omega_L [rad/s]')
title('Velocidad angular rueda izquierda')
legend('\omega_L')

subplot(212)
plot(t, wr, 'r', 'LineWidth', 2)
grid on
xlabel('Tiempo [s]')
ylabel('\omega_R [rad/s]')
title('Velocidad angular rueda derecha')
legend('\omega_R')