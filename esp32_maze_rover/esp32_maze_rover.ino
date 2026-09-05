// =============================================================================
// ESP32 — Maze Rover Controller
//
// UART <-> FPGA (9600 baud, 8N1 via Serial on GPIO 1/3)
// Sensors: BH1750 (I2C light), S12SD (analog UV), Soil Moisture (analog + servo)
// Web dashboard: WiFi AP mode, controls + live sensor data
//
// UART Protocol:
//   ESP32 -> FPGA:  0x01=START, 0x02=STOP, 0x03=RETURN
//   FPGA -> ESP32:  0xAA=MAZE_DONE, 0xBB=RUNNING, 0xCC=IDLE
// =============================================================================

#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <BH1750.h>
#include <ESP32Servo.h>

// =============================================================================
// PIN DEFINITIONS
// =============================================================================
#define UV_SENSOR_PIN   34
#define SOIL_SENSOR_PIN 35
#define SERVO_PIN       18

// =============================================================================
// UART COMMAND BYTES
// =============================================================================
#define CMD_START   0x01
#define CMD_STOP    0x02
#define CMD_RETURN  0x03

#define STATUS_MAZE_DONE  0xD1
#define STATUS_RUNNING    0xD2
#define STATUS_IDLE       0xD3

// =============================================================================
// WiFi AP Configuration
// =============================================================================
const char* AP_SSID     = "MazeRover";
const char* AP_PASSWORD = "rover1234";

// =============================================================================
// GLOBAL OBJECTS
// =============================================================================
WebServer server(80);
BH1750 lightSensor;
Servo soilServo;

// =============================================================================
// SENSOR DATA
// =============================================================================
float lightLux     = 0.0;
float uvIndex      = 0.0;
int   soilMoisture = 0;
bool  lightRead    = false;
bool  uvRead       = false;
bool  soilRead     = false;

// =============================================================================
// ROVER STATE
// =============================================================================
String roverState    = "IDLE";
bool   mazeCompleted = false;

// =============================================================================
// SERVO POSITIONS
// =============================================================================
#define SERVO_UP    60
#define SERVO_DOWN  150

// Non-blocking soil reading state machine
enum SoilPhase { SOIL_IDLE, SOIL_LOWERING, SOIL_RAISING };
SoilPhase soilPhase = SOIL_IDLE;
unsigned long soilTimer = 0;

// GPIO-level fallback for MAZE_DONE detection
// When FPGA maze_done is active, it holds TX line (GPIO 3) LOW
unsigned long gpioLowSince = 0;
bool gpioWasLow = false;

// =============================================================================
// SETUP
// =============================================================================
void setup() {
    // Pullup on RX to stabilize idle state against USB chip contention
    pinMode(3, INPUT_PULLUP);

    // Larger RX buffer to catch more heartbeat bytes
    Serial.setRxBufferSize(1024);

    // UART to FPGA at 9600 baud on GPIO 1 (TX) and GPIO 3 (RX)
    Serial.begin(9600);

    // Flush any boot garbage from the buffer
    delay(100);
    while (Serial.available()) Serial.read();

    // I2C + BH1750
    Wire.begin();
    lightSensor.begin(BH1750::CONTINUOUS_HIGH_RES_MODE);

    // Servo
    soilServo.attach(SERVO_PIN);
    soilServo.write(SERVO_UP);

    // ADC
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    // WiFi AP
    WiFi.softAP(AP_SSID, AP_PASSWORD);

    // Web server routes
    server.on("/",         HTTP_GET,  handleRoot);
    server.on("/data",     HTTP_GET,  handleData);
    server.on("/cmd",      HTTP_POST, handleCommand);
    server.on("/readlight", HTTP_POST, handleReadLight);
    server.on("/readuv",    HTTP_POST, handleReadUV);
    server.on("/readsoil",  HTTP_POST, handleReadSoil);
    server.onNotFound(handleNotFound);
    server.begin();
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
    server.handleClient();
    checkFPGAMessages();
    runSoilStateMachine();
    delay(5);
}

// =============================================================================
// CHECK UART FROM FPGA
// =============================================================================
void checkFPGAMessages() {
    // Method 1: Standard UART byte reception
    while (Serial.available()) {
        uint8_t b = Serial.read();

        if (b == STATUS_MAZE_DONE) {
            roverState    = "MAZE_DONE";
            mazeCompleted = true;
        }
        else if (b == STATUS_RUNNING) {
            roverState    = "RUNNING";
            mazeCompleted = false;
            lightRead = false;
            uvRead    = false;
            soilRead  = false;
        }
        else if (b == STATUS_IDLE) {
            roverState = "IDLE";
        }
        // All other bytes silently discarded
    }

    // Method 2: GPIO-level fallback for MAZE_DONE
    // When FPGA maze_done is active, it forces TX line LOW continuously.
    // Detect sustained LOW on GPIO 3 for >20ms = maze completed.
    // This works even if UART byte reception is completely broken.
    if (digitalRead(3) == LOW) {
        if (!gpioWasLow) {
            gpioLowSince = millis();
            gpioWasLow = true;
        } else if (millis() - gpioLowSince > 20) {
            // Sustained LOW for >20ms — FPGA is signaling maze_done
            if (roverState != "MAZE_DONE") {
                roverState    = "MAZE_DONE";
                mazeCompleted = true;
            }
        }
    } else {
        gpioWasLow = false;
    }
}

// =============================================================================
// INDIVIDUAL SENSOR READ HANDLERS
// =============================================================================
void handleReadLight() {
    lightLux = lightSensor.readLightLevel();
    lightRead = true;
    String json = "{\"light\":" + String(lightLux, 1) + "}";
    server.send(200, "application/json", json);
}

void handleReadUV() {
    int uvRaw = analogRead(UV_SENSOR_PIN);
    float uvVoltage = uvRaw * (3.3 / 4095.0);
    uvIndex = uvVoltage / 0.1;
    uvRead = true;
    String json = "{\"uvIndex\":" + String(uvIndex, 2) + "}";
    server.send(200, "application/json", json);
}

void handleReadSoil() {
    if (soilPhase != SOIL_IDLE) {
        server.send(409, "application/json", "{\"error\":\"Soil reading in progress\"}");
        return;
    }
    // Start non-blocking soil sequence
    soilServo.write(SERVO_DOWN);
    soilTimer = millis();
    soilPhase = SOIL_LOWERING;
    server.send(200, "application/json", "{\"status\":\"reading\"}");
}

void runSoilStateMachine() {
    switch (soilPhase) {
        case SOIL_IDLE:
            break;
        case SOIL_LOWERING:
            if (millis() - soilTimer >= 3000) {
                int soilRaw = analogRead(SOIL_SENSOR_PIN);
                soilMoisture = map(soilRaw, 4095, 0, 0, 100);
                soilMoisture = constrain(soilMoisture, 0, 100);
                soilServo.write(SERVO_UP);
                soilTimer = millis();
                soilPhase = SOIL_RAISING;
            }
            break;
        case SOIL_RAISING:
            if (millis() - soilTimer >= 1000) {
                soilRead = true;
                soilPhase = SOIL_IDLE;
            }
            break;
    }
}

// =============================================================================
// HTTP: COMMAND HANDLER
// =============================================================================
void handleCommand() {
    if (!server.hasArg("cmd")) {
        server.send(400, "application/json", "{\"error\":\"Missing cmd\"}");
        return;
    }
    String cmd = server.arg("cmd");

    if (cmd == "start") {
        Serial.write(CMD_START);
        roverState    = "RUNNING";
        mazeCompleted = false;
        lightRead = false; uvRead = false; soilRead = false;
        server.send(200, "application/json", "{\"status\":\"ok\",\"action\":\"start\"}");
    }
    else if (cmd == "stop") {
        Serial.write(CMD_STOP);
        roverState = "IDLE";
        server.send(200, "application/json", "{\"status\":\"ok\",\"action\":\"stop\"}");
    }
    else if (cmd == "return") {
        Serial.write(CMD_RETURN);
        roverState    = "RETURNING";
        mazeCompleted = false;
        server.send(200, "application/json", "{\"status\":\"ok\",\"action\":\"return\"}");
    }
    else {
        server.send(400, "application/json", "{\"error\":\"Unknown command\"}");
    }
}

// =============================================================================
// HTTP: SENSOR DATA
// =============================================================================
void handleData() {
    String json = "{";
    json += "\"roverState\":\"" + roverState + "\",";
    json += "\"mazeCompleted\":" + String(mazeCompleted ? "true" : "false") + ",";
    json += "\"lightRead\":" + String(lightRead ? "true" : "false") + ",";
    json += "\"uvRead\":" + String(uvRead ? "true" : "false") + ",";
    json += "\"soilRead\":" + String(soilRead ? "true" : "false") + ",";
    json += "\"soilBusy\":" + String(soilPhase != SOIL_IDLE ? "true" : "false") + ",";
    json += "\"light\":" + String(lightLux, 1) + ",";
    json += "\"uvIndex\":" + String(uvIndex, 2) + ",";
    json += "\"soilMoisture\":" + String(soilMoisture);
    json += "}";
    server.send(200, "application/json", json);
}

// =============================================================================
// HTTP: 404
// =============================================================================
void handleNotFound() {
    server.send(404, "text/plain", "Not Found");
}

// =============================================================================
// HTTP: DASHBOARD PAGE
// =============================================================================
void handleRoot() {
    String html = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Maze Rover Dashboard</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', system-ui, -apple-system, sans-serif;
            background: linear-gradient(135deg, #0f0c29, #302b63, #24243e);
            color: #e0e0e0;
            min-height: 100vh;
            padding: 20px;
        }
        .container { max-width: 700px; margin: 0 auto; }
        h1 {
            text-align: center; font-size: 2em; margin-bottom: 10px;
            background: linear-gradient(90deg, #00d2ff, #3a7bd5);
            -webkit-background-clip: text; -webkit-text-fill-color: transparent;
            background-clip: text;
        }
        .subtitle { text-align: center; color: #888; margin-bottom: 30px; font-size: 0.9em; }

        .status-card {
            background: rgba(255,255,255,0.05); backdrop-filter: blur(10px);
            border: 1px solid rgba(255,255,255,0.1); border-radius: 16px;
            padding: 24px; margin-bottom: 20px;
        }
        .status-row {
            display: flex; justify-content: space-between;
            align-items: center; margin-bottom: 12px;
        }
        .status-label { color: #aaa; font-size: 0.95em; }
        .status-value {
            font-weight: 600; font-size: 1.1em;
            padding: 4px 14px; border-radius: 20px;
        }
        .state-IDLE       { background: #555; color: #ccc; }
        .state-RUNNING    { background: #1b5e20; color: #69f0ae; }
        .state-MAZE_DONE  { background: #e65100; color: #ffcc80; }
        .state-RETURNING  { background: #0d47a1; color: #82b1ff; }

        .maze-badge {
            display: inline-block; padding: 6px 18px;
            border-radius: 20px; font-weight: 600; font-size: 0.95em;
        }
        .maze-yes { background: #00c853; color: #000; animation: pulse 1.5s infinite; }
        .maze-no  { background: #424242; color: #999; }
        @keyframes pulse {
            0%, 100% { box-shadow: 0 0 0 0 rgba(0,200,83,0.4); }
            50%      { box-shadow: 0 0 0 12px rgba(0,200,83,0); }
        }

        .controls { display: flex; gap: 12px; margin-bottom: 20px; }
        .btn {
            flex: 1; padding: 14px; border: none; border-radius: 12px;
            font-size: 1em; font-weight: 600; cursor: pointer;
            transition: transform 0.15s, box-shadow 0.15s; color: #fff;
        }
        .btn:hover { transform: translateY(-2px); box-shadow: 0 6px 20px rgba(0,0,0,0.3); }
        .btn:active { transform: translateY(0); }
        .btn-start  { background: linear-gradient(135deg, #00c853, #00e676); color: #000; }
        .btn-stop   { background: linear-gradient(135deg, #d32f2f, #ff5252); }
        .btn-return { background: linear-gradient(135deg, #1565c0, #42a5f5); }
        .btn:disabled { opacity: 0.4; cursor: not-allowed; transform: none; }

        .sensors-grid {
            display: grid; grid-template-columns: 1fr 1fr 1fr;
            gap: 12px; margin-bottom: 20px;
        }
        .sensor-card {
            background: rgba(255,255,255,0.05);
            border: 1px solid rgba(255,255,255,0.08);
            border-radius: 14px; padding: 20px 16px; text-align: center;
        }
        .sensor-icon { font-size: 2em; margin-bottom: 8px; }
        .sensor-value { font-size: 1.8em; font-weight: 700; margin-bottom: 4px; }
        .sensor-unit { color: #888; font-size: 0.85em; }
        .sensor-label { color: #aaa; font-size: 0.8em; margin-top: 6px; }

        .btn-sensor {
            display: block; width: 100%; margin-top: 10px;
            padding: 8px 12px; border: none; border-radius: 8px;
            font-size: 0.85em; font-weight: 600; cursor: pointer;
            background: linear-gradient(135deg, #7c4dff, #651fff);
            color: #fff; transition: transform 0.15s, opacity 0.15s;
        }
        .btn-sensor:hover { transform: translateY(-1px); opacity: 0.9; }
        .btn-sensor:active { transform: translateY(0); }
        .btn-sensor:disabled { opacity: 0.4; cursor: not-allowed; transform: none; }
        .btn-sensor.reading {
            background: linear-gradient(135deg, #ff6f00, #ffa000);
            animation: blink 0.8s infinite;
        }
        @keyframes blink {
            0%, 100% { opacity: 1; }
            50% { opacity: 0.5; }
        }

        .footer { text-align: center; color: #555; font-size: 0.8em; margin-top: 20px; }
        @media (max-width: 500px) {
            .sensors-grid { grid-template-columns: 1fr; }
            .controls { flex-direction: column; }
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>&#x1F916; Maze Rover</h1>
        <p class="subtitle">FPGA + ESP32 Autonomous Maze Solver</p>

        <div class="status-card">
            <div class="status-row">
                <span class="status-label">Rover State</span>
                <span id="roverState" class="status-value state-IDLE">IDLE</span>
            </div>
            <div class="status-row">
                <span class="status-label">Maze Status</span>
                <span id="mazeBadge" class="maze-badge maze-no">Not Completed</span>
            </div>
        </div>

        <div class="controls">
            <button class="btn btn-start" id="btnStart" onclick="sendCmd('start')">&#x25B6; Start</button>
            <button class="btn btn-stop" id="btnStop" onclick="sendCmd('stop')">&#x23F9; Stop</button>
            <button class="btn btn-return" id="btnReturn" onclick="sendCmd('return')" disabled>&#x21A9; Return</button>
        </div>

        <div class="sensors-grid">
            <div class="sensor-card">
                <div class="sensor-icon">&#x2600;</div>
                <div class="sensor-value" id="lightVal">--</div>
                <div class="sensor-unit">lux</div>
                <div class="sensor-label">Light (BH1750)</div>
                <button class="btn-sensor" id="btnLight" onclick="readSensor('light')">&#x1F4D6; Read Light</button>
            </div>
            <div class="sensor-card">
                <div class="sensor-icon">&#x1F31E;</div>
                <div class="sensor-value" id="uvVal">--</div>
                <div class="sensor-unit">UV Index</div>
                <div class="sensor-label">UV (S12SD)</div>
                <button class="btn-sensor" id="btnUV" onclick="readSensor('uv')">&#x1F4D6; Read UV</button>
            </div>
            <div class="sensor-card">
                <div class="sensor-icon">&#x1F4A7;</div>
                <div class="sensor-value" id="soilVal">--</div>
                <div class="sensor-unit">%</div>
                <div class="sensor-label">Soil Moisture</div>
                <button class="btn-sensor" id="btnSoil" onclick="readSensor('soil')">&#x1F4D6; Read Soil</button>
            </div>
        </div>

        <p class="footer">Auto-refreshes every 1s &bull; WiFi: MazeRover</p>
    </div>

    <script>
        function sendCmd(cmd) {
            fetch('/cmd', {
                method: 'POST',
                headers: {'Content-Type': 'application/x-www-form-urlencoded'},
                body: 'cmd=' + cmd
            })
            .then(r => r.json())
            .then(d => { fetchData(); })
            .catch(e => console.error('CMD error:', e));
        }

        function readSensor(type) {
            let url, btnId, valId;
            if (type === 'light') { url = '/readlight'; btnId = 'btnLight'; valId = 'lightVal'; }
            else if (type === 'uv') { url = '/readuv'; btnId = 'btnUV'; valId = 'uvVal'; }
            else if (type === 'soil') { url = '/readsoil'; btnId = 'btnSoil'; valId = 'soilVal'; }

            const btn = document.getElementById(btnId);
            btn.disabled = true;
            btn.classList.add('reading');
            btn.textContent = '⏳ Reading...';

            fetch(url, { method: 'POST' })
            .then(r => r.json())
            .then(d => {
                if (type === 'light' && d.light !== undefined) {
                    document.getElementById(valId).textContent = d.light.toFixed(1);
                } else if (type === 'uv' && d.uvIndex !== undefined) {
                    document.getElementById(valId).textContent = d.uvIndex.toFixed(1);
                } else if (type === 'soil') {
                    // Soil is non-blocking, poll for result
                    btn.textContent = '⏳ Lowering servo...';
                    setTimeout(pollSoil, 4500);
                    return;
                }
                btn.disabled = false;
                btn.classList.remove('reading');
                btn.textContent = '✓ Done! Read Again';
                setTimeout(() => { btn.textContent = '📖 Read ' + capitalize(type); }, 3000);
            })
            .catch(e => {
                console.error('Sensor error:', e);
                btn.disabled = false;
                btn.classList.remove('reading');
                btn.textContent = '❌ Error! Retry';
            });
        }

        function pollSoil() {
            fetch('/data').then(r => r.json()).then(d => {
                const btn = document.getElementById('btnSoil');
                if (d.soilRead) {
                    document.getElementById('soilVal').textContent = d.soilMoisture;
                    btn.disabled = false;
                    btn.classList.remove('reading');
                    btn.textContent = '✓ Done! Read Again';
                    setTimeout(() => { btn.textContent = '📖 Read Soil'; }, 3000);
                } else if (d.soilBusy) {
                    btn.textContent = '⏳ Measuring...';
                    setTimeout(pollSoil, 1000);
                } else {
                    btn.disabled = false;
                    btn.classList.remove('reading');
                    btn.textContent = '📖 Read Soil';
                }
            });
        }

        function capitalize(s) { return s.charAt(0).toUpperCase() + s.slice(1); }

        function fetchData() {
            fetch('/data')
            .then(r => r.json())
            .then(d => {
                const stateEl = document.getElementById('roverState');
                stateEl.textContent = d.roverState;
                stateEl.className = 'status-value state-' + d.roverState;

                const badge = document.getElementById('mazeBadge');
                if (d.mazeCompleted) {
                    badge.textContent = '✓ Maze Completed';
                    badge.className = 'maze-badge maze-yes';
                } else {
                    badge.textContent = 'Not Completed';
                    badge.className = 'maze-badge maze-no';
                }

                document.getElementById('btnReturn').disabled = !d.mazeCompleted;

                // Update sensor values if read
                if (d.lightRead) {
                    document.getElementById('lightVal').textContent = d.light.toFixed(1);
                }
                if (d.uvRead) {
                    document.getElementById('uvVal').textContent = d.uvIndex.toFixed(1);
                }
                if (d.soilRead) {
                    document.getElementById('soilVal').textContent = d.soilMoisture;
                }
            })
            .catch(e => console.error('Data fetch error:', e));
        }

        setInterval(fetchData, 1000);
        fetchData();
    </script>
</body>
</html>
)rawliteral";

    server.send(200, "text/html", html);
}
