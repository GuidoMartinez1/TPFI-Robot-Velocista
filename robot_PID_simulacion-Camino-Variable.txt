% Parámetros iniciales
dt = 0.05; % Intervalo de tiempo (s)
tiempo_total = 10; % Duración de la simulación (s)
num_pasos = tiempo_total / dt;

% Setpoint dinámico (camino sinusoidal y luego recto)
tiempo = linspace(0, tiempo_total, num_pasos);
frecuencia = 0.5; % Frecuencia de la onda sinusoidal
amplitud = 5;     % Amplitud de la onda sinusoidal
tiempo_cambio = 5; % Tiempo en el que cambia de sinusoidal a recto

% Generar el setpoint dinámico
setpoint = zeros(1, num_pasos);
for i = 1:num_pasos
    if tiempo(i) <= tiempo_cambio
        setpoint(i) = amplitud * sin(2 * pi * frecuencia * tiempo(i)); % Camino sinusoidal
    else
        setpoint(i) = setpoint(find(tiempo <= tiempo_cambio, 1, 'last')); % Camino recto
    end
end

bandaerrorsup = setpoint + 0.5; % Banda de error superior
bandaerrorinf = setpoint - 0.5; % Banda de error inferior

% Controlador PID
Kp = 30;  % Ganancia proporcional
Ki = 16;  % Ganancia integral ajustada
Kd = 0.1;  % Ganancia derivativa ajustada

% Límite del término integral
limite_integral = 7.5;

% Variables del sistema
posicion = 0; % Posición inicial del robot
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
    perturbacion = (rand * 0.1) - 0.05; % Pequeñas perturbaciones aleatorias
    posicion = posicion + salida_pid * dt + perturbacion; 
    
    % Guardar valores
    posiciones(i) = posicion;
    derivada_anterior = error;
end

% Calcular el error en estado estable (últimos 20% de los datos)
indice_estado_estable = round(0.8 * num_pasos):num_pasos;
error_promedio_estable = mean(errores(indice_estado_estable));
fprintf('Error en estado estable promedio: %.4f\n', error_promedio_estable);

% Visualización del seguimiento de la curva
figure;

% Gráfico 1: Posición y Setpoint
subplot(2,1,1); % Gráfico principal: Posición y Setpoint
hold on;
plot(tiempo, setpoint, 'r--', 'DisplayName', 'Pista a seguir');
plot(tiempo, bandaerrorsup, 'c-', 'DisplayName', 'Banda de error superior');
plot(tiempo, bandaerrorinf, 'c-', 'DisplayName', 'Banda de error inferior');
plot(tiempo, posiciones, 'b-', 'DisplayName', 'Posición del robot');
title('Simulación de seguimiento de curva (sinusoidal y luego recto)');
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
