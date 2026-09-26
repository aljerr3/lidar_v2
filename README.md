# Escáner LiDAR 3D v2.0

Escáner LiDAR 3D DIY con ESP32, sensor TF-Luna y dos servos TD-8120MG (azimut y elevación).

Video: https://www.youtube.com/watch?v=uK7RXH-Pjhw

## Archivos

| Archivo | Descripción |
|---|---|
| `lidar_v2.ino` | Firmware para ESP32 (edita `ssid` y `password` con tu red WiFi antes de cargarlo) |
| `3d/Lidar_servo.step` | Diseño 3D completo (STEP) |
| `3d/Lidar_servo_step.zip` | El mismo STEP comprimido (descarga más liviana) |
| `docs/Manual_Escaner_LiDAR_3D.pdf` | Manual de armado y uso |
| `docs/Esquematico_LiDAR.png` | Esquemático de conexiones |
| `docs/Diagrama_cableado_LiDAR.png` | Diagrama de cableado |

## Conexiones

| Componente | ESP32 |
|---|---|
| TF-Luna TXD | GPIO 16 (RX2) |
| TF-Luna RXD | GPIO 17 (TX2) |
| Servo azimut (señal) | GPIO 15 |
| Servo elevación (señal) | GPIO 4 |
| Pulsador (a GND) | GPIO 27 |

- Alimentación: fuente externa de 5 V ≥ 3 A (recomendado 5 A) para servos, TF-Luna y VIN del ESP32. No alimentes los servos desde el USB.
- Todas las GND a un punto común.
- Condensador de 1000 µF / 16 V entre +5 V y GND, cerca de los servos.
- TF-Luna pin 5 (CFG) sin conectar = modo UART.
