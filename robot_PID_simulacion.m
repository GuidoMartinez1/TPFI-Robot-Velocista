%El código simula un controlador PID (Proporcional, Integral, Derivativo) que controla la posición de un robot para que siga
% una referencia (setpoint) que aumenta linealmente con el tiempo. 
% El objetivo del controlador es minimizar el error entre la posición real del robot y el setpoint.


% Parámetros iniciales
dt = 0.05; % Intervalo de tiempo (s)
tiempo_total = 10; % Duración de la simulación (s)
num_pasos = tiempo_total / dt;

% Setpoint dinámico (curva deseada)
tiempo = linspace(0, tiempo_total, num_pasos);
setpoint = tiempo; % Setpoint como una línea creciente
bandaerrorsup = setpoint + 0.5; % Banda de error superior
bandaerrorinf = setpoint - 0.5; % Banda de error inferior

%El setpoint es la trayectoria que el robot debe seguir. Aquí, es una línea recta creciente.
%Se define una banda de error de 0.5 arriba y abajo del setpoint. Esto sirve para visualizar si el robot se desvía mucho.


% Controlador PID
Kp = 3.8;  % Ganancia proporcional
Ki = 0.9; % Ganancia integral ajustada
Kd = 0.2;  % Ganancia derivativa ajustada

% Límite del término integral
limite_integral = 7.5;

%Kp (proporcional): Corrige el error actual.
%Ki (integral): Corrige el error acumulado en el tiempo.
%Kd (derivativo): Corrige la velocidad del cambio del error.
%El límite del término integral evita que el controlador acumule errores excesivos (un fenómeno conocido como wind-up).


% Variables del sistema
posicion = 1; % Posición inicial del robot
derivada_anterior = 0;
integral = 0;
posiciones = zeros(1, num_pasos); % Almacenar la trayectoria
errores = zeros(1, num_pasos); % Almacenar errores en cada paso

% Simulación
for i = 1:num_pasos
    % Error actual
    error = setpoint(i) - posicion;
    errores(i) = error; % Guardar el error en cada paso
    
    % Cálculo PID
    integral = integral + error * dt;
    % Limitar el término integral
    if integral > limite_integral
        integral = limite_integral;
    elseif integral < -limite_integral
        integral = -limite_integral;
    end
    derivada = (error - derivada_anterior) / dt;
    salida_pid = Kp * error + Ki * integral + Kd * derivada;
    
    % Actualizar posición (aplicación de control + perturbación)
    perturbacion = (rand * 0.1) - 0.1;
    posicion = posicion + salida_pid * dt;% + perturbacion;
    
    % Guardar valores
    posiciones(i) = posicion;
    derivada_anterior = error;
end

%Error: Diferencia entre el setpoint (lo deseado) y la posición actual del robot.
%PID:
%Se calcula la integral del error acumulando valores.
%Se calcula la derivada para medir qué tan rápido cambia el error.
%Se combina todo con ​Kp, Kd y Ki para calcular la salida de control.
%La posición del robot se actualiza sumando la salida del controlador multiplicada por dt
%Se añade una pequeña perturbación aleatoria para simular un sistema real con ruido.


% Calcular el error en estado estable (últimos 20% de los datos)
%Sirve para medir qué tan bien el robot sigue la referencia al final de la simulación.
indice_estado_estable = round(0.8 * num_pasos):num_pasos;
error_promedio_estable = mean(errores(indice_estado_estable));
fprintf('Error en estado estable promedio: %.4f\n', error_promedio_estable);

% Visualización del seguimiento de la curva
figure;
subplot(2,1,1); % Gráfico 1: Posición y Setpoint
hold on;
plot(tiempo, setpoint, 'r--', 'DisplayName', 'Pista a seguir');
plot(tiempo, bandaerrorsup, 'c-', 'DisplayName', 'Banda de error superior');
plot(tiempo, bandaerrorinf, 'c-', 'DisplayName', 'Banda de error inferior');
plot(tiempo, posiciones, 'b-', 'DisplayName', 'Posición del robot');
title('Simulación de seguimiento de curva (visto desde arriba)');
xlabel('Tiempo (s)');
ylabel('Posición');
legend;
grid on;
hold off;

% Gráfico 2: Error en cada instante de tiempo
subplot(2,1,2);
plot(tiempo, errores, 'm-', 'DisplayName', 'Error');
yline(error_promedio_estable, 'k--', 'DisplayName', 'Error Promedio Estable');
title('Error en el tiempo');
xlabel('Tiempo (s)');
ylabel('Error');
legend;
grid on;
