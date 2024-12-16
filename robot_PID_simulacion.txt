% Parámetros iniciales
dt = 0.05; % Intervalo de tiempo (s)
tiempo_total = 10; % Duración de la simulación (s)
num_pasos = tiempo_total / dt;

% Setpoint dinámico (curva deseada)
tiempo = linspace(0, tiempo_total, num_pasos);
setpoint = tiempo; % Setpoint como una línea creciente
bandaerrorsup = tiempo + 0.5; % Banda de error superior
bandaerrorinf = tiempo - 0.5; % Banda de error inferior

% Controlador PID
Kp = 3.8;  % Ganancia proporcional
Ki = 0.9; % Ganancia integral ajustada
Kd = 0.2;  % Ganancia derivativa ajustada

% Límite del término integral
limite_integral = 7.5;

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
subplot(2,1,1); % Gráfico 1: Posición y Setpoint
hold on;
plot(tiempo, setpoint, 'r--', 'DisplayName', 'Pista a seguir');
plot(tiempo, bandaerrorsup, 'c-');
plot(tiempo, bandaerrorinf, 'c-', 'DisplayName', 'Banda de error');
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
