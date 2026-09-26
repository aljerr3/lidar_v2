# EscÃ¡ner LiDAR 3D v2.0

EscÃ¡ner LiDAR 3D DIY con ESP32, sensor TF-Luna y dos servos TD-8120MG (azimut y elevaciÃ³n).

Video: https://www.youtube.com/watch?v=uK7RXH-Pjhw

## Archivos

| Archivo | DescripciÃ³n |
|---|---|
| `lidar_v2.ino` | Firmware para ESP32 (edita `ssid` y `password` con tu red WiFi antes de cargarlo) |
| `3d/Lidar_servo.step` | DiseÃ±o 3D completo (STEP) |
| `docs/Manual_Escaner_LiDAR_3D.pdf` | Manual de armado y uso |
| `docs/Esquematico_LiDAR.png` | EsquemÃ¡tico de conexiones |
| `docs/Diagrama_cableado_LiDAR.png` | Diagrama de cableado |

## Conexiones

| Componente | ESP32 |
|---|---|
| TF-Luna TXD | GPIO 16 (RX2) |
| TF-Luna RXD | GPIO 17 (TX2) |
| Servo azimut (seÃ±al) | GPIO 15 |
| Servo elevaciÃ³n (seÃ±al) | GPIO 4 |
| Pulsador (a GND) | GPIO 27 |

- AlimentaciÃ³n: fuente externa de 5 V â‰¥ 3 A (recomendado 5 A) para servos, TF-Luna y VIN del ESP32. No alimentes los servos desde el USB.
- Todas las GND a un punto comÃºn.
- Condensador de 1000 ÂµF / 16 V entre +5 V y GND, cerca de los servos.
- TF-Luna pin 5 (CFG) sin conectar = modo UART.
