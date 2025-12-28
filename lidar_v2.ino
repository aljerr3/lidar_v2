/*
 * LIDAR 3D SCANNER v2.0
 * ESP32 + TF-Luna (UART) + 2x TD-8120MG Servos
 * Con modo PRECISO y MANUAL configurable
 */

#include <ESP32Servo.h>
#include <WiFi.h>
#include <WebServer.h>

// ==================== CONFIGURACIÓN WIFI ====================
const char* ssid = "Rodriguez";
const char* password = "arqui2021";

// ==================== PINES ====================
#define SERVO_AZIMUTH_PIN 15
#define SERVO_ELEVACION_PIN 4
#define BUTTON_PIN 27

// ==================== TF-LUNA UART ====================
HardwareSerial TFLuna(2);

// ==================== PARÁMETROS DE ESCANEO ====================
#define AZ_MIN 0
#define AZ_MAX 180
#define EL_MIN 0
#define EL_MAX 90

// ==================== OBJETOS ====================
Servo servoAzimuth;
Servo servoElevacion;
WebServer server(80);

// ==================== VARIABLES ====================
volatile bool escaneoActivo = false;
volatile int modoEscaneo = 0;          // 0 = preciso, 1 = manual
volatile int azimuthActual = 0;
volatile int elevacionActual = 0;
volatile unsigned long puntosCapturados = 0;
volatile unsigned long tiempoInicio = 0;
volatile bool escaneoCompletado = false;

// ==================== PARÁMETROS MODO MANUAL ====================
int manualAzStep = 2;
int manualElStep = 2;
int manualMuestras = 2;
int manualDelayServo = 20;
int manualDelayLectura = 10;

// ==================== DATOS ====================
struct Punto {
  uint8_t az;
  uint8_t el;
  uint16_t d;
  uint16_t amp;
};

struct LecturaLidar {
  int16_t distancia;
  int16_t amplitud;
  bool valida;
};

const int MAX_PUNTOS = 17000;  // Reducido para caber en RAM sin PSRAM
Punto* puntos = nullptr;

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n========================================");
  Serial.println("   LIDAR 3D SCANNER v2.0");
  Serial.println("========================================\n");

  Serial.printf("RAM libre: %d bytes\n", ESP.getFreeHeap());
  Serial.printf("Intentando asignar %d bytes para %d puntos...\n", MAX_PUNTOS * sizeof(Punto), MAX_PUNTOS);

  puntos = (Punto*)malloc(MAX_PUNTOS * sizeof(Punto));
  if (puntos == nullptr) {
    Serial.println("ERROR: Sin memoria suficiente!");
    Serial.println("Reduce MAX_PUNTOS en el codigo");
    while(1) { delay(1000); }
  }
  Serial.printf("OK! RAM libre despues: %d bytes\n", ESP.getFreeHeap());

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  TFLuna.begin(115200, SERIAL_8N1, 16, 17);
  delay(100);

  servoAzimuth.setPeriodHertz(50);
  servoElevacion.setPeriodHertz(50);
  servoAzimuth.attach(SERVO_AZIMUTH_PIN, 400, 2600);
  servoElevacion.attach(SERVO_ELEVACION_PIN, 500, 2500);

  irHome();
  conectarWiFi();
  configurarRutas();
  server.begin();

  xTaskCreatePinnedToCore(TaskEscaneo, "Escaneo", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(TaskWebServer, "WebServer", 8192, NULL, 1, NULL, 0);

  Serial.println("Sistema listo\n");
}

void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    procesarComando(cmd);
  }
}

// ==================== TAREAS ====================
void TaskWebServer(void* param) {
  for (;;) {
    server.handleClient();
    delay(5);
  }
}

void TaskEscaneo(void* param) {
  for (;;) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      delay(50);
      if (digitalRead(BUTTON_PIN) == LOW) {
        while (digitalRead(BUTTON_PIN) == LOW) delay(10);
        escaneoActivo = !escaneoActivo;
      }
    }

    if (escaneoActivo) {
      ejecutarEscaneo();
    }

    delay(50);
  }
}

// ==================== ESCANEO ====================
void ejecutarEscaneo() {
  escaneoCompletado = false;
  puntosCapturados = 0;
  tiempoInicio = millis();

  int elStep = (modoEscaneo == 1) ? manualElStep : 1;
  int numLineas = (EL_MAX - EL_MIN) / elStep + 1;
  String modoNombre = (modoEscaneo == 1) ? "MANUAL" : "PRECISO";

  Serial.println("\n========================================");
  Serial.printf("   ESCANEO %s\n", modoNombre.c_str());
  if (modoEscaneo == 1) {
    Serial.printf("   Az: %d | El: %d | Muestras: %d\n", manualAzStep, manualElStep, manualMuestras);
  }
  Serial.println("========================================\n");

  servoAzimuth.write(AZ_MIN);
  servoElevacion.write(EL_MIN);
  delay(1000);

  for (int linea = 0; linea < numLineas && escaneoActivo; linea++) {
    int elevacion = EL_MIN + (linea * elStep);
    elevacionActual = elevacion;

    servoElevacion.write(elevacion);
    delay(200);

    servoAzimuth.write(AZ_MIN);
    delay(300);

    if (modoEscaneo == 1) {
      barridoLineaManual(AZ_MIN, AZ_MAX, elevacion);
    } else {
      barridoLineaPreciso(AZ_MIN, AZ_MAX, elevacion);
    }

    float progreso = (linea + 1) * 100.0 / numLineas;
    Serial.printf("Linea %2d/%d | El: %2d | Puntos: %lu | %.0f%%\n",
                  linea + 1, numLineas, elevacion, puntosCapturados, progreso);
  }

  escaneoActivo = false;
  escaneoCompletado = true;

  Serial.println("\n========================================");
  Serial.printf("COMPLETADO: %lu puntos\n", puntosCapturados);
  Serial.println("========================================\n");

  irHome();
}

void barridoLineaManual(int desde, int hasta, int elevacion) {
  // Limpiar buffer UART antes de cada línea
  while (TFLuna.available()) TFLuna.read();

  for (int pos = desde; pos <= hasta && escaneoActivo; pos += manualAzStep) {
    servoAzimuth.write(pos);
    azimuthActual = pos;

    // Delay proporcional al paso: más grados = más tiempo
    int delayCalculado = manualAzStep * 15;
    if (delayCalculado < manualDelayServo) delayCalculado = manualDelayServo;
    delay(delayCalculado);

    // Limpiar buffer antes de leer (evita lecturas viejas)
    while (TFLuna.available()) TFLuna.read();
    delay(12);

    // Promediar N lecturas
    int32_t sumaD = 0, sumaAmp = 0;
    int lecturas = 0;
    for (int j = 0; j < manualMuestras; j++) {
      LecturaLidar lectura = leerTFLuna();
      if (lectura.valida && lectura.distancia > 0 && lectura.distancia < 8000) {
        sumaD += lectura.distancia;
        sumaAmp += lectura.amplitud;
        lecturas++;
      }
    }

    if (lecturas > 0) {
      guardarPunto(pos, elevacion, sumaD / lecturas, sumaAmp / lecturas);
    }
  }
}

void barridoLineaPreciso(int desde, int hasta, int elevacion) {
  // Limpiar buffer UART antes de cada línea
  while (TFLuna.available()) TFLuna.read();

  for (int pos = desde; pos <= hasta && escaneoActivo; pos++) {
    servoAzimuth.write(pos);
    azimuthActual = pos;
    delay(15);

    int32_t sumaD = 0, sumaAmp = 0;
    int lecturas = 0;
    for (int j = 0; j < 3; j++) {
      LecturaLidar lectura = leerTFLuna();
      if (lectura.valida && lectura.distancia > 0 && lectura.distancia < 8000) {
        sumaD += lectura.distancia;
        sumaAmp += lectura.amplitud;
        lecturas++;
      }
    }

    if (lecturas > 0) {
      guardarPunto(pos, elevacion, sumaD / lecturas, sumaAmp / lecturas);
    }
  }
}

// ==================== TF-LUNA ====================
LecturaLidar leerTFLuna() {
  LecturaLidar lectura = {-1, 0, false};
  uint8_t buf[9];
  unsigned long start = millis();

  while (TFLuna.available() && TFLuna.peek() != 0x59) {
    TFLuna.read();
  }

  while (TFLuna.available() < 9) {
    if (millis() - start > 50) return lectura;
  }

  TFLuna.readBytes(buf, 9);

  if (buf[0] != 0x59 || buf[1] != 0x59) return lectura;

  uint8_t sum = 0;
  for (int i = 0; i < 8; i++) sum += buf[i];
  if (sum != buf[8]) return lectura;

  lectura.distancia = buf[2] | (buf[3] << 8);
  lectura.amplitud = buf[4] | (buf[5] << 8);

  if (lectura.amplitud < 100) return lectura;

  lectura.valida = true;
  return lectura;
}

// ==================== DATOS ====================
void guardarPunto(int azimuth, int elevacion, int16_t distancia, int16_t amplitud) {
  if (puntosCapturados < MAX_PUNTOS) {
    puntos[puntosCapturados].az = azimuth;
    puntos[puntosCapturados].el = elevacion;
    puntos[puntosCapturados].d = distancia;
    puntos[puntosCapturados].amp = amplitud;
    puntosCapturados++;
  }
}

// ==================== UTILIDADES ====================
void irHome() {
  servoAzimuth.write(AZ_MIN);
  servoElevacion.write(EL_MIN);
  azimuthActual = AZ_MIN;
  elevacionActual = EL_MIN;
  delay(500);
}

void procesarComando(String cmd) {
  if (cmd == "start") { modoEscaneo = 0; escaneoActivo = true; }
  else if (cmd == "manual") { modoEscaneo = 1; escaneoActivo = true; }
  else if (cmd == "stop") { escaneoActivo = false; }
  else if (cmd == "home") { irHome(); }
  else if (cmd == "status") {
    Serial.printf("Az: %d | El: %d | Puntos: %lu\n", azimuthActual, elevacionActual, puntosCapturados);
  }
}

void conectarWiFi() {
  Serial.print("Conectando WiFi");
  WiFi.begin(ssid, password);

  int intentos = 0;
  while (WiFi.status() != WL_CONNECTED && intentos < 20) {
    delay(500);
    Serial.print(".");
    intentos++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf(" OK\nIP: %s\n", WiFi.localIP().toString().c_str());
  } else {
    Serial.println(" FALLO");
  }
}

// ==================== SERVIDOR WEB ====================
void configurarRutas() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/data", HTTP_GET, handleData);

  server.on("/start", HTTP_GET, []() {
    if (!escaneoActivo) { modoEscaneo = 0; escaneoActivo = true; }
    server.send(200, "text/plain", "OK");
  });

  server.on("/startmanual", HTTP_GET, []() {
    if (!escaneoActivo) { modoEscaneo = 1; escaneoActivo = true; }
    server.send(200, "text/plain", "OK");
  });

  server.on("/config", HTTP_GET, []() {
    if (server.hasArg("az")) manualAzStep = constrain(server.arg("az").toInt(), 1, 10);
    if (server.hasArg("el")) manualElStep = constrain(server.arg("el").toInt(), 1, 10);
    if (server.hasArg("samples")) manualMuestras = constrain(server.arg("samples").toInt(), 1, 10);
    if (server.hasArg("delay")) manualDelayServo = constrain(server.arg("delay").toInt(), 5, 100);
    if (server.hasArg("delayRead")) manualDelayLectura = constrain(server.arg("delayRead").toInt(), 5, 50);

    String json = "{\"az\":" + String(manualAzStep) +
                  ",\"el\":" + String(manualElStep) +
                  ",\"samples\":" + String(manualMuestras) +
                  ",\"delay\":" + String(manualDelayServo) +
                  ",\"delayRead\":" + String(manualDelayLectura) + "}";
    server.send(200, "application/json", json);
  });

  server.on("/stop", HTTP_GET, []() {
    escaneoActivo = false;
    server.send(200, "text/plain", "OK");
  });

  server.on("/home", HTTP_GET, []() {
    irHome();
    server.send(200, "text/plain", "OK");
  });

  server.on("/clear", HTTP_GET, []() {
    puntosCapturados = 0;
    server.send(200, "text/plain", "OK");
  });

  server.on("/status", HTTP_GET, []() {
    String json = "{\"az\":" + String(azimuthActual) +
                  ",\"el\":" + String(elevacionActual) +
                  ",\"puntos\":" + String(puntosCapturados) +
                  ",\"activo\":" + String(escaneoActivo ? "true" : "false") +
                  ",\"completado\":" + String(escaneoCompletado ? "true" : "false") +
                  ",\"modo\":" + String(modoEscaneo) +
                  ",\"manualAz\":" + String(manualAzStep) +
                  ",\"manualEl\":" + String(manualElStep) +
                  ",\"manualSamples\":" + String(manualMuestras) +
                  ",\"manualDelay\":" + String(manualDelayServo) + "}";
    server.send(200, "application/json", json);
  });
}

void handleData() {
  WiFiClient client = server.client();

  client.println("HTTP/1.1 200 OK");
  client.println("Content-Type: application/json");
  client.println("Transfer-Encoding: chunked");
  client.println("Connection: close");
  client.println();

  String chunk = "[";

  for (unsigned long i = 0; i < puntosCapturados; i++) {
    if (i > 0) chunk += ",";
    chunk += "{\"az\":" + String(puntos[i].az) +
             ",\"el\":" + String(puntos[i].el) +
             ",\"d\":" + String(puntos[i].d) +
             ",\"amp\":" + String(puntos[i].amp) + "}";

    if (chunk.length() > 1000) {
      client.printf("%X\r\n", chunk.length());
      client.print(chunk);
      client.print("\r\n");
      chunk = "";
      yield();
    }
  }

  chunk += "]";
  client.printf("%X\r\n", chunk.length());
  client.print(chunk);
  client.print("\r\n");
  client.print("0\r\n\r\n");
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>LiDAR 3D Scanner</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <script src="https://unpkg.com/three@0.128.0/build/three.min.js"></script>
  <script src="https://unpkg.com/three@0.128.0/examples/js/controls/OrbitControls.js"></script>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { font-family: Arial, sans-serif; background: #0a0a1a; overflow: hidden; }

    #panel {
      position: absolute;
      top: 15px;
      left: 15px;
      background: rgba(15, 15, 30, 0.95);
      padding: 20px;
      border-radius: 15px;
      color: #fff;
      z-index: 100;
      min-width: 240px;
      border: 1px solid rgba(78, 205, 196, 0.3);
    }

    h2 { margin-bottom: 15px; font-size: 18px; color: #4ecdc4; }

    .btn {
      width: 100%;
      padding: 10px;
      margin: 4px 0;
      border: none;
      border-radius: 8px;
      cursor: pointer;
      font-size: 12px;
      font-weight: 600;
      text-transform: uppercase;
      color: #fff;
    }

    .btn:hover { opacity: 0.9; transform: translateY(-2px); }

    .btn-start { background: linear-gradient(135deg, #4ecdc4, #44a08d); }
    .btn-manual { background: linear-gradient(135deg, #f093fb, #f5576c); }
    .btn-stop { background: linear-gradient(135deg, #ff6b6b, #ee5a5a); }
    .btn-home { background: linear-gradient(135deg, #667eea, #764ba2); }
    .btn-clear { background: linear-gradient(135deg, #ffa726, #ff7043); }
    .btn-load { background: linear-gradient(135deg, #45b7d1, #2980b9); }
    .btn-export { background: linear-gradient(135deg, #27ae60, #229954); }
    .btn-apply { background: linear-gradient(135deg, #9b59b6, #8e44ad); }

    .config-section {
      margin-top: 15px;
      padding: 12px;
      background: rgba(0,0,0,0.3);
      border-radius: 8px;
    }

    .config-section h3 {
      font-size: 11px;
      color: #f093fb;
      margin-bottom: 10px;
      text-transform: uppercase;
    }

    .config-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin: 6px 0;
    }

    .config-row label { font-size: 11px; color: #aaa; }

    .config-row input {
      width: 55px;
      padding: 5px;
      border: 1px solid #444;
      border-radius: 4px;
      background: #1a1a2e;
      color: #fff;
      text-align: center;
    }

    #stats {
      margin-top: 15px;
      padding: 12px;
      background: rgba(0,0,0,0.4);
      border-radius: 8px;
      font-size: 12px;
    }

    #stats p { margin: 6px 0; display: flex; justify-content: space-between; }
    #stats span { color: #4ecdc4; font-weight: bold; }

    .status-scanning { color: #4ecdc4 !important; animation: pulse 1s infinite; }
    .status-done { color: #4caf50 !important; }

    @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }

    #progress { margin-top: 10px; height: 6px; background: rgba(255,255,255,0.1); border-radius: 3px; }
    #progressBar { height: 100%; background: linear-gradient(90deg, #4ecdc4, #44a08d); width: 0%; transition: width 0.3s; }
  </style>
</head>
<body>

<div id="panel">
  <h2>LiDAR 3D Scanner <span style="font-size:10px;color:#666;">v4</span></h2>

  <button class="btn btn-start" onclick="startScan()">PRECISO</button>
  <button class="btn btn-manual" onclick="startManual()">MANUAL</button>
  <button class="btn btn-stop" onclick="stopScan()">DETENER</button>
  <button class="btn btn-home" onclick="goHome()">HOME</button>
  <button class="btn btn-clear" onclick="clearAll()">LIMPIAR</button>
  <button class="btn btn-load" onclick="forceLoad()">CARGAR</button>
  <button class="btn btn-export" onclick="exportPLY()">EXPORTAR PLY</button>
  <button class="btn btn-apply" onclick="exportSTL()">EXPORTAR STL</button>

  <div class="config-section">
    <h3>Visualizacion</h3>
    <div class="config-row">
      <label>Colorear por:</label>
      <select id="colorMode" onchange="updateVisualization()" style="padding:5px;border-radius:4px;background:#1a1a2e;color:#fff;border:1px solid #444;">
        <option value="distance">Distancia</option>
        <option value="amplitude">Amplitud</option>
        <option value="elevation">Elevacion</option>
      </select>
    </div>
    <div class="config-row">
      <label>Mostrar como:</label>
      <select id="viewMode" onchange="updateVisualization()" style="padding:5px;border-radius:4px;background:#1a1a2e;color:#fff;border:1px solid #444;">
        <option value="points">Puntos</option>
        <option value="mesh">Mesh</option>
        <option value="wireframe">Wireframe</option>
      </select>
    </div>
  </div>

  <div class="config-section">
    <h3>Filtros</h3>
    <div class="config-row">
      <label>Umbral ruido (cm):</label>
      <input type="number" id="filterThreshold" value="100" min="10" max="500" style="width:55px;padding:5px;border:1px solid #444;border-radius:4px;background:#1a1a2e;color:#fff;text-align:center;">
    </div>
    <button class="btn btn-stop" onclick="filterNoise()">FILTRAR RUIDO</button>
    <div class="config-row" style="margin-top:10px;">
      <label>Max dist mesh (cm):</label>
      <input type="number" id="meshMaxDist" value="50" min="10" max="500" style="width:55px;padding:5px;border:1px solid #444;border-radius:4px;background:#1a1a2e;color:#fff;text-align:center;">
    </div>
    <p style="font-size:10px;color:#888;margin-top:5px;">No une puntos con diferencia mayor</p>
  </div>

  <div class="config-section">
    <h3>Configuracion Manual</h3>
    <div class="config-row">
      <label>Paso Azimuth (grados):</label>
      <input type="number" id="cfgAz" value="2" min="1" max="10">
    </div>
    <div class="config-row">
      <label>Paso Elevacion (grados):</label>
      <input type="number" id="cfgEl" value="2" min="1" max="10">
    </div>
    <div class="config-row">
      <label>Muestras por punto:</label>
      <input type="number" id="cfgSamples" value="2" min="1" max="10">
    </div>
    <div class="config-row">
      <label>Delay Servo (ms):</label>
      <input type="number" id="cfgDelay" value="20" min="5" max="100">
    </div>
    <div class="config-row">
      <label>Delay Lectura (ms):</label>
      <input type="number" id="cfgDelayRead" value="10" min="5" max="50">
    </div>
    <button class="btn btn-apply" onclick="applyConfig()">APLICAR</button>
  </div>

  <div id="stats">
    <p>Estado: <span id="status">Listo</span></p>
    <p>Posicion: <span id="position">0 / 0</span></p>
    <p>Puntos: <span id="points">0</span></p>
  </div>

  <div id="progress"><div id="progressBar"></div></div>
</div>

<script>
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x0a0a1a);

const camera = new THREE.PerspectiveCamera(75, window.innerWidth / window.innerHeight, 1, 50000);
camera.position.set(400, 400, 400);

const renderer = new THREE.WebGLRenderer({ antialias: true });
renderer.setSize(window.innerWidth, window.innerHeight);
document.body.appendChild(renderer.domElement);

const controls = new THREE.OrbitControls(camera, renderer.domElement);
controls.enableDamping = true;

scene.add(new THREE.GridHelper(2000, 40, 0x333355, 0x222233));
scene.add(new THREE.AxesHelper(200));

let allPoints = [];
let pointsMesh;
let solidMesh;
let autoRefreshEnabled = true;
let fetchInterval = null;
const pointsMaterial = new THREE.PointsMaterial({ size: 2, vertexColors: true });

function stopAutoRefresh() {
  console.log('>>> STOP AUTO REFRESH');
  autoRefreshEnabled = false;
  if (fetchInterval) { clearInterval(fetchInterval); fetchInterval = null; }
}

function startAutoRefresh() {
  console.log('>>> START AUTO REFRESH');
  autoRefreshEnabled = true;
  if (!fetchInterval) { fetchInterval = setInterval(fetchData, 500); }
}

function updateVisualization() {
  const viewMode = document.getElementById('viewMode').value;
  if (viewMode === 'points') {
    updatePointCloud();
  } else {
    updateMesh(viewMode === 'wireframe');
  }
}

function updatePointCloud() {
  if (allPoints.length === 0) return;

  const colorMode = document.getElementById('colorMode').value;
  const positions = new Float32Array(allPoints.length * 3);
  const colors = new Float32Array(allPoints.length * 3);

  // Calcular rangos segun modo
  let minVal = Infinity, maxVal = 0;
  allPoints.forEach(p => {
    let val;
    if (colorMode === 'amplitude') val = p.amp || 0;
    else if (colorMode === 'elevation') val = p.el;
    else val = p.d;
    if (val < minVal) minVal = val;
    if (val > maxVal) maxVal = val;
  });
  const range = maxVal - minVal || 1;

  allPoints.forEach((p, i) => {
    const az = (p.az - 90) * Math.PI / 180;
    const el = (90 - p.el) * Math.PI / 180;
    const r = p.d;

    const rHoriz = r * Math.cos(el);

    positions[i * 3] = rHoriz * Math.sin(az);
    positions[i * 3 + 1] = r * Math.sin(el);
    positions[i * 3 + 2] = rHoriz * Math.cos(az);

    // Colorear segun modo seleccionado
    let val;
    if (colorMode === 'amplitude') val = p.amp || 0;
    else if (colorMode === 'elevation') val = p.el;
    else val = p.d;

    const t = (val - minVal) / range;
    const hue = (1 - t) * 0.6;
    const rgb = hslToRgb(hue, 1, 0.5);
    colors[i * 3] = rgb[0];
    colors[i * 3 + 1] = rgb[1];
    colors[i * 3 + 2] = rgb[2];
  });

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));

  if (pointsMesh) scene.remove(pointsMesh);
  if (solidMesh) scene.remove(solidMesh);
  pointsMesh = new THREE.Points(geometry, pointsMaterial);
  scene.add(pointsMesh);
}

function updateMesh(wireframe) {
  if (allPoints.length < 10) return;

  // Limpiar meshes anteriores
  if (pointsMesh) scene.remove(pointsMesh);
  if (solidMesh) scene.remove(solidMesh);

  const colorMode = document.getElementById('colorMode').value;

  // Organizar puntos en grilla por elevacion y azimuth
  const grid = {};
  allPoints.forEach(p => {
    const key = p.el;
    if (!grid[key]) grid[key] = [];
    grid[key].push(p);
  });

  // Ordenar cada fila por azimuth
  const elevations = Object.keys(grid).map(Number).sort((a, b) => a - b);
  elevations.forEach(el => {
    grid[el].sort((a, b) => a.az - b.az);
  });

  // Calcular rangos para color
  let minVal = Infinity, maxVal = 0;
  allPoints.forEach(p => {
    let val = colorMode === 'amplitude' ? (p.amp || 0) : colorMode === 'elevation' ? p.el : p.d;
    if (val < minVal) minVal = val;
    if (val > maxVal) maxVal = val;
  });
  const range = maxVal - minVal || 1;

  // Funcion para convertir punto a coordenadas 3D
  function toXYZ(p) {
    const az = (p.az - 90) * Math.PI / 180;
    const el = (90 - p.el) * Math.PI / 180;
    const rHoriz = p.d * Math.cos(el);
    return {
      x: rHoriz * Math.sin(az),
      y: p.d * Math.sin(el),
      z: rHoriz * Math.cos(az)
    };
  }

  // Funcion para obtener color
  function getColor(p) {
    let val = colorMode === 'amplitude' ? (p.amp || 0) : colorMode === 'elevation' ? p.el : p.d;
    const t = (val - minVal) / range;
    return hslToRgb((1 - t) * 0.6, 1, 0.5);
  }

  // Crear triangulos conectando puntos vecinos
  const positions = [];
  const colors = [];
  const maxDistDiff = parseInt(document.getElementById('meshMaxDist').value) || 50;

  for (let i = 0; i < elevations.length - 1; i++) {
    const row1 = grid[elevations[i]];
    const row2 = grid[elevations[i + 1]];

    for (let j = 0; j < row1.length - 1; j++) {
      const p1 = row1[j];
      const p2 = row1[j + 1];

      const p3 = row2.find(p => Math.abs(p.az - p1.az) <= 2);
      const p4 = row2.find(p => Math.abs(p.az - p2.az) <= 2);

      if (p3 && p4) {
        // Verificar que los puntos no esten muy separados en distancia
        const d12 = Math.abs(p1.d - p2.d);
        const d13 = Math.abs(p1.d - p3.d);
        const d24 = Math.abs(p2.d - p4.d);
        const d34 = Math.abs(p3.d - p4.d);

        const v1 = toXYZ(p1), v2 = toXYZ(p2), v3 = toXYZ(p3), v4 = toXYZ(p4);
        const c1 = getColor(p1), c2 = getColor(p2), c3 = getColor(p3), c4 = getColor(p4);

        // Triangulo 1: p1, p2, p3 - solo si los 3 puntos estan cerca
        if (d12 < maxDistDiff && d13 < maxDistDiff) {
          positions.push(v1.x, v1.y, v1.z, v2.x, v2.y, v2.z, v3.x, v3.y, v3.z);
          colors.push(c1[0], c1[1], c1[2], c2[0], c2[1], c2[2], c3[0], c3[1], c3[2]);
        }

        // Triangulo 2: p2, p4, p3 - solo si los 3 puntos estan cerca
        if (d24 < maxDistDiff && d34 < maxDistDiff) {
          positions.push(v2.x, v2.y, v2.z, v4.x, v4.y, v4.z, v3.x, v3.y, v3.z);
          colors.push(c2[0], c2[1], c2[2], c4[0], c4[1], c4[2], c3[0], c3[1], c3[2]);
        }
      }
    }
  }

  if (positions.length === 0) {
    console.log('No se pudieron crear triangulos');
    updatePointCloud();
    return;
  }

  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  geometry.computeVertexNormals();

  const material = new THREE.MeshBasicMaterial({
    vertexColors: true,
    side: THREE.DoubleSide,
    wireframe: wireframe
  });

  solidMesh = new THREE.Mesh(geometry, material);
  scene.add(solidMesh);
}

function hslToRgb(h, s, l) {
  let r, g, b;
  if (s === 0) { r = g = b = l; }
  else {
    const hue2rgb = (p, q, t) => {
      if (t < 0) t += 1; if (t > 1) t -= 1;
      if (t < 1/6) return p + (q - p) * 6 * t;
      if (t < 1/2) return q;
      if (t < 2/3) return p + (q - p) * (2/3 - t) * 6;
      return p;
    };
    const q = l < 0.5 ? l * (1 + s) : l + s - l * s;
    const p = 2 * l - q;
    r = hue2rgb(p, q, h + 1/3);
    g = hue2rgb(p, q, h);
    b = hue2rgb(p, q, h - 1/3);
  }
  return [r, g, b];
}

let lastPointCount = 0;

function fetchData() {
  if (!autoRefreshEnabled) {
    console.log('fetchData BLOCKED - autoRefresh disabled');
    return;
  }
  fetch('/data').then(r => r.json()).then(data => {
    if (Array.isArray(data) && data.length !== lastPointCount) {
      console.log('fetchData: loading ' + data.length + ' points');
      allPoints = data;
      lastPointCount = data.length;
      updateVisualization();
    }
  }).catch(e => {});
}

function fetchStatus() {
  fetch('/status').then(r => r.json()).then(s => {
    const statusEl = document.getElementById('status');
    if (s.activo) {
      statusEl.textContent = s.modo === 1 ? 'Manual...' : 'Preciso...';
      statusEl.className = 'status-scanning';
    } else if (s.completado) {
      statusEl.textContent = 'Completado';
      statusEl.className = 'status-done';
    } else {
      statusEl.textContent = 'Listo';
      statusEl.className = '';
    }
    document.getElementById('position').textContent = s.az + ' / ' + s.el;
    document.getElementById('points').textContent = s.puntos;

    const elStep = s.modo === 1 ? s.manualEl : 1;
    const totalLineas = Math.floor(90 / elStep) + 1;
    const lineaActual = Math.floor(s.el / elStep);
    const progreso = s.activo ? (lineaActual / totalLineas * 100) : (s.completado ? 100 : 0);
    document.getElementById('progressBar').style.width = progreso + '%';
  }).catch(e => {});
}

function startScan() {
  startAutoRefresh();
  allPoints = []; lastPointCount = 0;
  if (pointsMesh) scene.remove(pointsMesh);
  if (solidMesh) scene.remove(solidMesh);
  fetch('/clear').then(() => fetch('/start'));
}

function startManual() {
  startAutoRefresh();
  allPoints = []; lastPointCount = 0;
  if (pointsMesh) scene.remove(pointsMesh);
  if (solidMesh) scene.remove(solidMesh);
  applyConfig();
  setTimeout(() => fetch('/clear').then(() => fetch('/startmanual')), 100);
}

function applyConfig() {
  const az = document.getElementById('cfgAz').value;
  const el = document.getElementById('cfgEl').value;
  const samples = document.getElementById('cfgSamples').value;
  const delay = document.getElementById('cfgDelay').value;
  const delayRead = document.getElementById('cfgDelayRead').value;
  fetch('/config?az=' + az + '&el=' + el + '&samples=' + samples + '&delay=' + delay + '&delayRead=' + delayRead);
}

function stopScan() { fetch('/stop'); }
function goHome() { fetch('/home'); }
function clearAll() {
  allPoints = []; lastPointCount = 0;
  if (pointsMesh) scene.remove(pointsMesh);
  if (solidMesh) scene.remove(solidMesh);
  fetch('/clear');
}

function forceLoad() {
  startAutoRefresh();
  lastPointCount = 0;
  fetch('/data').then(r => r.json()).then(data => {
    if (Array.isArray(data) && data.length > 0) {
      allPoints = data;
      lastPointCount = data.length;
      updateVisualization();
      alert('Cargados ' + data.length + ' puntos');
    } else {
      alert('No hay datos');
    }
  }).catch(e => alert('Error: ' + e));
}

function exportPLY() {
  if (allPoints.length === 0) { alert('No hay puntos'); return; }

  const colorMode = document.getElementById('colorMode').value;
  let ply = 'ply\nformat ascii 1.0\nelement vertex ' + allPoints.length + '\n';
  ply += 'property float x\nproperty float y\nproperty float z\n';
  ply += 'property uchar red\nproperty uchar green\nproperty uchar blue\nend_header\n';

  // Calcular rangos segun modo de color
  let minVal = Infinity, maxVal = 0;
  allPoints.forEach(p => {
    let val;
    if (colorMode === 'amplitude') val = p.amp || 0;
    else if (colorMode === 'elevation') val = p.el;
    else val = p.d;
    if (val < minVal) minVal = val;
    if (val > maxVal) maxVal = val;
  });
  const range = maxVal - minVal || 1;

  allPoints.forEach(p => {
    const az = (p.az - 90) * Math.PI / 180;
    const el = (90 - p.el) * Math.PI / 180;
    const r = p.d;
    const rHoriz = r * Math.cos(el);
    const x = rHoriz * Math.sin(az);
    const y = r * Math.sin(el);
    const z = rHoriz * Math.cos(az);

    let val;
    if (colorMode === 'amplitude') val = p.amp || 0;
    else if (colorMode === 'elevation') val = p.el;
    else val = p.d;

    const t = (val - minVal) / range;
    const rgb = hslToRgb((1 - t) * 0.6, 1, 0.5);

    ply += x.toFixed(2) + ' ' + y.toFixed(2) + ' ' + z.toFixed(2) + ' ';
    ply += Math.floor(rgb[0] * 255) + ' ' + Math.floor(rgb[1] * 255) + ' ' + Math.floor(rgb[2] * 255) + '\n';
  });

  const blob = new Blob([ply], { type: 'text/plain' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'scan_' + Date.now() + '.ply';
  a.click();
}

function exportSTL() {
  if (allPoints.length < 10) { alert('Necesitas mas puntos para generar mesh'); return; }

  // Organizar puntos en grilla
  const grid = {};
  allPoints.forEach(p => {
    if (!grid[p.el]) grid[p.el] = [];
    grid[p.el].push(p);
  });

  const elevations = Object.keys(grid).map(Number).sort((a, b) => a - b);
  elevations.forEach(el => grid[el].sort((a, b) => a.az - b.az));

  function toXYZ(p) {
    const az = (p.az - 90) * Math.PI / 180;
    const el = (90 - p.el) * Math.PI / 180;
    const rHoriz = p.d * Math.cos(el);
    return { x: rHoriz * Math.sin(az), y: p.d * Math.sin(el), z: rHoriz * Math.cos(az) };
  }

  // Generar triangulos
  const triangles = [];
  const maxDistDiff = parseInt(document.getElementById('meshMaxDist').value) || 50;

  for (let i = 0; i < elevations.length - 1; i++) {
    const row1 = grid[elevations[i]];
    const row2 = grid[elevations[i + 1]];
    for (let j = 0; j < row1.length - 1; j++) {
      const p1 = row1[j], p2 = row1[j + 1];
      const p3 = row2.find(p => Math.abs(p.az - p1.az) <= 2);
      const p4 = row2.find(p => Math.abs(p.az - p2.az) <= 2);
      if (p3 && p4) {
        const d12 = Math.abs(p1.d - p2.d);
        const d13 = Math.abs(p1.d - p3.d);
        const d24 = Math.abs(p2.d - p4.d);
        const d34 = Math.abs(p3.d - p4.d);

        const v1 = toXYZ(p1), v2 = toXYZ(p2), v3 = toXYZ(p3), v4 = toXYZ(p4);

        if (d12 < maxDistDiff && d13 < maxDistDiff) {
          triangles.push([v1, v2, v3]);
        }
        if (d24 < maxDistDiff && d34 < maxDistDiff) {
          triangles.push([v2, v4, v3]);
        }
      }
    }
  }

  if (triangles.length === 0) { alert('No se pudo generar mesh'); return; }

  // Crear STL ASCII
  let stl = 'solid scan\n';
  triangles.forEach(tri => {
    // Calcular normal
    const u = { x: tri[1].x - tri[0].x, y: tri[1].y - tri[0].y, z: tri[1].z - tri[0].z };
    const v = { x: tri[2].x - tri[0].x, y: tri[2].y - tri[0].y, z: tri[2].z - tri[0].z };
    const n = {
      x: u.y * v.z - u.z * v.y,
      y: u.z * v.x - u.x * v.z,
      z: u.x * v.y - u.y * v.x
    };
    const len = Math.sqrt(n.x*n.x + n.y*n.y + n.z*n.z) || 1;
    n.x /= len; n.y /= len; n.z /= len;

    stl += '  facet normal ' + n.x.toFixed(6) + ' ' + n.y.toFixed(6) + ' ' + n.z.toFixed(6) + '\n';
    stl += '    outer loop\n';
    tri.forEach(v => {
      stl += '      vertex ' + v.x.toFixed(4) + ' ' + v.y.toFixed(4) + ' ' + v.z.toFixed(4) + '\n';
    });
    stl += '    endloop\n';
    stl += '  endfacet\n';
  });
  stl += 'endsolid scan\n';

  const blob = new Blob([stl], { type: 'text/plain' });
  const a = document.createElement('a');
  a.href = URL.createObjectURL(blob);
  a.download = 'scan_' + Date.now() + '.stl';
  a.click();
  alert('Exportado: ' + triangles.length + ' triangulos');
}

function filterNoise() {
  if (allPoints.length < 10) { alert('No hay suficientes puntos'); return; }

  stopAutoRefresh();  // Detener completamente el auto-refresh
  const threshold = parseInt(document.getElementById('filterThreshold').value) || 100;
  const originalCount = allPoints.length;

  // Organizar puntos en grilla por elevacion
  const grid = {};
  allPoints.forEach((p, idx) => {
    p._idx = idx;
    if (!grid[p.el]) grid[p.el] = [];
    grid[p.el].push(p);
  });

  // Ordenar cada fila por azimuth
  const elevations = Object.keys(grid).map(Number).sort((a, b) => a - b);
  elevations.forEach(el => grid[el].sort((a, b) => a.az - b.az));

  // Marcar puntos a eliminar
  const toRemove = new Set();

  elevations.forEach((el, elIdx) => {
    const row = grid[el];
    const prevRow = elIdx > 0 ? grid[elevations[elIdx - 1]] : null;
    const nextRow = elIdx < elevations.length - 1 ? grid[elevations[elIdx + 1]] : null;

    row.forEach((p, i) => {
      let validNeighbors = 0;
      let totalDiff = 0;

      // Vecino izquierdo (misma fila)
      if (i > 0) {
        const diff = Math.abs(p.d - row[i - 1].d);
        if (diff < threshold) validNeighbors++;
        totalDiff += diff;
      }

      // Vecino derecho (misma fila)
      if (i < row.length - 1) {
        const diff = Math.abs(p.d - row[i + 1].d);
        if (diff < threshold) validNeighbors++;
        totalDiff += diff;
      }

      // Vecino arriba (fila anterior)
      if (prevRow) {
        const neighbor = prevRow.find(n => Math.abs(n.az - p.az) <= 2);
        if (neighbor) {
          const diff = Math.abs(p.d - neighbor.d);
          if (diff < threshold) validNeighbors++;
          totalDiff += diff;
        }
      }

      // Vecino abajo (fila siguiente)
      if (nextRow) {
        const neighbor = nextRow.find(n => Math.abs(n.az - p.az) <= 2);
        if (neighbor) {
          const diff = Math.abs(p.d - neighbor.d);
          if (diff < threshold) validNeighbors++;
          totalDiff += diff;
        }
      }

      // Si tiene menos de 2 vecinos validos, es ruido
      if (validNeighbors < 2) {
        toRemove.add(p._idx);
      }
    });
  });

  // Filtrar puntos
  allPoints = allPoints.filter((p, idx) => !toRemove.has(idx));

  const removed = originalCount - allPoints.length;
  lastPointCount = allPoints.length;
  updateVisualization();

  alert('Eliminados ' + removed + ' puntos de ruido\nQuedan ' + allPoints.length + ' puntos');
}

fetchInterval = setInterval(fetchData, 500);
setInterval(fetchStatus, 300);

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);
}
animate();

window.addEventListener('resize', () => {
  camera.aspect = window.innerWidth / window.innerHeight;
  camera.updateProjectionMatrix();
  renderer.setSize(window.innerWidth, window.innerHeight);
});
</script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}
