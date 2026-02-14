Este repositorio contiene el desarrollo de una plataforma experimental de simulación para un dron multirrotor. El objetivo del proyecto es el aprendizaje técnico y la validación de sistemas de Guiado, Navegación y Control (GNC) utilizando metodologías de diseño basado en modelos.

🛠️ Módulos y Funcionalidades
El desarrollo abarcó la implementación de las siguientes funcionalidades técnicas:

Entorno de Simulación: Vinculación con Unreal Engine para visualización 3D y soporte para entradas Human-in-the-Loop mediante hardware externo (Joystick).

Dinámica de Vuelo: Modelado de ecuaciones de movimiento de cuerpo rígido (6DOF) y caracterización básica de sistemas de propulsión y aerodinámica.

Arquitectura de Control: Diseño de lazos PID en cascada con soporte para modos de vuelo Acro, Stabilizer y GPS.

Navegación y Sensores: Modelado de ruido en sensores (IMU, GPS, Barómetro) y algoritmos para la estimación de actitud y posición.

Interfaz y Software: Paneles de telemetría para monitoreo de estados y configuración de ejecución Multi-rate para los diferentes lazos del sistema.

Validación SIL: Verificación del sistema mediante simulación Software-in-the-Loop para asegurar la integridad de la lógica de control.

📂 Estructura del Proyecto
Documentation/: Informe técnico del proyecto en formato PDF.

Models/: Archivos de Simulink para la planta (Drone.slx) y el sistema de control (FCS_SIL.slx).

Scripts/: Rutinas de inicialización de parámetros (Init.m) y registro de datos (Logger.m).

S-Function/: Binario compilado (Flight_sf.mexw64) utilizado para la ejecución de la lógica de vuelo.

📋 Requisitos
Para ejecutar la simulación se requiere MATLAB/Simulink y los siguientes módulos:

Aerospace Blockset & Toolbox.

Control System & Simulink Control Design.

Computer Vision & Image Processing Toolbox.

Simulink 3D Animation & Simulink Coder.

📺 Demostración
El video demostrativo de la simulación con el audio original se encuentra disponible en el siguiente enlace:
https://drive.google.com/drive/folders/1-hqnKSXZZtOtCEmlY9cFQOxKplq6wV9h?usp=sharing
