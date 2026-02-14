%% ------------------------------------------------------------------------
%% SCRIPT DE INICIALIZACIÓN: PROYETO DRONE
%% Descripción: Definición de parámetros físicos, arquitectura temporal
%%              y configuración de sensores para simulación.
%% ------------------------------------------------------------------------
clear; clc; close all;
disp('-> Cargando parámetros del sistema...');

%% 1. CONFIGURACIÓN DE ARQUITECTURA (MIL/SIL)
% -------------------------------------------------------------------------
% HIL_MODE: Selección de Planta (Modelo Matemático vs Hardware Real)
% 0 = Math Plant (Simulación)
% 1 = HIL (Sensores Reales)
HIL_MODE = 0; 

% SIL_MODE: Implementación del Algoritmo de Control
% 0 = Model-in-the-Loop (Bloques Simulink)
% 1 = Software-in-the-Loop (Código C++ Generado)
SIL_MODE = 1; 

fprintf('-> Configuración activa: HIL=%d | SIL=%d\n', HIL_MODE, SIL_MODE);

%% 2. DEFINICIÓN DE TIEMPOS DE MUESTREO (MULTI-RATE)
% -------------------------------------------------------------------------
% Frecuencias de operación [Hz]
Sim.Freq.Fisica        = 1000;  % Solver de ecuaciones dinámicas (ODE)
Sim.Freq.Control_Rate  = 1000;  % Lazo de Control de Tasa Angular
Sim.Freq.Control_Angle = 500;   % Lazo de Control de Actitud
Sim.Freq.Control_Pos   = 50;    % Lazo de Navegación/Posición
Sim.Freq.Supervisor    = 100;   % Lógica de Estados y Seguridad
Sim.Freq.Video         = 50;    % Tasa de actualización gráfica

% Frecuencias de muestreo de sensores [Hz]
Sim.Freq.Sensor_IMU    = 1000;  
Sim.Freq.Sensor_Baro   = 50;    
Sim.Freq.Sensor_Mag    = 50;    
Sim.Freq.Sensor_GPS    = 10;    

% Cálculo de Sample Times [s]
Sim.Ts_fisica   = 1 / Sim.Freq.Fisica;
Sim.Ts_control  = 1 / Sim.Freq.Control_Rate;
Sim.Ts_angle    = 1 / Sim.Freq.Control_Angle;
Sim.Ts_pos      = 1 / Sim.Freq.Control_Pos;
Sim.Ts_video    = 1 / Sim.Freq.Video;
Sim.Ts_logic    = 1 / Sim.Freq.Supervisor;

% Sample Times de Sensores
Sim.Ts_imu      = 1 / Sim.Freq.Sensor_IMU;
Sim.Ts_baro     = 1 / Sim.Freq.Sensor_Baro;
Sim.Ts_mag      = 1 / Sim.Freq.Sensor_Mag;
Sim.Ts_gps      = 1 / Sim.Freq.Sensor_GPS;

% Inicialización de Flags
Sim.Modo_Vuelo_Init = 0; % 0: Stabilize, 1: Acro
Sim.System_Ready    = 0;

%% 3. MODELO FÍSICO DEL VEHÍCULO
% -------------------------------------------------------------------------
% 3.1. Inercia y Masa
Drone.Fisica.Masa = 2.0;          % [kg]
Drone.Fisica.g    = 9.81;         % [m/s^2]
Drone.Fisica.Ixx  = 0.035;        % [kg*m^2]
Drone.Fisica.Iyy  = 0.035;        % [kg*m^2]
Drone.Fisica.Izz  = 0.065;        % [kg*m^2]
Drone.Fisica.InertiaMatrix = diag([Drone.Fisica.Ixx, Drone.Fisica.Iyy, Drone.Fisica.Izz]);

% 3.2. Geometría (Configuración X)
Drone.Geom.d_x = 0.159;           % Brazo en eje X [m]
Drone.Geom.d_y = 0.159;           % Brazo en eje Y [m]
Drone.Geom.L   = sqrt(Drone.Geom.d_x^2 + Drone.Geom.d_y^2); 

% 3.3. Aerodinámica
Drone.Aero.Cd = diag([0.030, 0.030, 0.060]);   % Coeficientes de Drag Traslacional
Drone.Aero.Kd = diag([0.01, 0.01, 0.05]);      % Coeficientes de Drag Rotacional
Drone.Aero.M_Flap = [0, -0.02, 0; 0.02, 0, 0; 0, 0, 0]; % Matriz de Flapping

% 3.4. Sistema de Propulsión
Drone.Motor.MaxThrust = 9.81;     % [N] Empuje máximo por motor
Drone.Motor.MinThrust = 0.2;      % [N] Empuje en idle
Drone.Motor.Ct = 1.0;             % Coeficiente de empuje (Adimensionalizado)
Drone.Motor.Cm = 0.015;           % Coeficiente de momento

%% 4. SISTEMA DE ENERGÍA Y SENSORES
% -------------------------------------------------------------------------
% 4.1. Modelo de Batería (LiPo 4S)
Drone.Battery.Cells         = 4;            
Drone.Battery.Capacity_Ah   = 5.0;          
Drone.Battery.R_internal    = 0.012;        % [Ohms]
Drone.Battery.I_base        = 0.5;          % Corriente base (Aviónica)
Drone.Battery.I_max_motor   = 20.0;         % Corriente max por motor
Drone.Battery.MinVolts_Prot = 3.0 * Drone.Battery.Cells;

% Curvas de Descarga (Lookup Table)
Drone.Battery.DischargeCurve_Cap = [0, 0.01, 0.02, 0.05, 0.1, 0.5, 0.9, 1.0];
Drone.Battery.DischargeCurve_Vol = [2.7, 2.85, 3.0, 3.2, 3.5, 3.8, 4.1, 4.2] * Drone.Battery.Cells;

% 4.2. Caracterización de Ruido en Sensores (Varianza sigma^2)
% Sensor de Voltaje
Drone.Sensors.Voltage.Sigma      = 0.05;    
Drone.Sensors.Voltage.NoisePower = (Drone.Sensors.Voltage.Sigma^2) * Sim.Ts_logic;
Drone.Sensors.Voltage.Bits       = 12;      
Drone.Sensors.Voltage.V_ref      = 18.0;    
Drone.Sensors.Voltage.Quantum    = Drone.Sensors.Voltage.V_ref / (2^Drone.Sensors.Voltage.Bits);

% Sensor de Corriente
Drone.Sensors.Current.Sigma      = 0.5;     
Drone.Sensors.Current.NoisePower = (Drone.Sensors.Current.Sigma^2) * Sim.Ts_logic;
Drone.Sensors.Current.Bits       = 12;      
Drone.Sensors.Current.I_max      = 100.0; 
Drone.Sensors.Current.Quantum    = Drone.Sensors.Current.I_max / (2^Drone.Sensors.Current.Bits);

% IMU (Giroscopio y Acelerómetro)
Drone.Sensors.Gyro.Sigma       = 0.002; 
Drone.Sensors.Gyro.NoisePower  = (Drone.Sensors.Gyro.Sigma^2) * Sim.Ts_imu;
Drone.Sensors.Accel.Sigma      = 0.1241; 
Drone.Sensors.Accel.NoisePower = (Drone.Sensors.Accel.Sigma^2) * Sim.Ts_imu;

% Magnetómetro y Barómetro
Drone.Sensors.Mag.Sigma        = 0.002;
Drone.Sensors.Mag.NoisePower   = (Drone.Sensors.Mag.Sigma^2) * Sim.Ts_mag;
Drone.Sensors.Baro.Sigma       = 0.15;
Drone.Sensors.Baro.NoisePower  = (Drone.Sensors.Baro.Sigma^2) * Sim.Ts_baro;

% GPS
Drone.Sensors.GPS.Pos_Sigma      = 2.0; 
Drone.Sensors.GPS.Vel_Sigma      = 0.1;
Drone.Sensors.GPS.Pos_NoisePower = (Drone.Sensors.GPS.Pos_Sigma^2) * Sim.Ts_gps;
Drone.Sensors.GPS.Vel_NoisePower = (Drone.Sensors.GPS.Vel_Sigma^2) * Sim.Ts_gps;

%% 5. VARIABLES DE ENTORNO Y CONDICIONES INICIALES
% -------------------------------------------------------------------------
Env.Viento.Vel_Base_NED   = [0; 0; 0]; 
Env.Viento.k_turb         = 0;       
Env.Viento.Intensidad_Max = 15;      

Env.Pos.Init_NED = [0, 0, -0.01];

%% 6. VISUALIZACIÓN
% -------------------------------------------------------------------------
try
    % Aplicación de paleta de colores para visualización de Sample Times
    miEstilo = simulink.sampletimecolors.Palette('EngineeringStyle');
    miEstilo.DiscreteSampleTimeColors = ["#FF0000", "#FFA500", "#0000FF", "#008000"];
    simulink.sampletimecolors.applyPalette(miEstilo);
catch
    % Fallback silencioso si la librería no está disponible
end

disp('-> Inicialización completada exitosamente.');