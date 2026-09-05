# Farmware-Bot 🌾🤖

> **FPGA-based Autonomous Precision Agriculture Rover & Environmental Monitor**  
> *Hardware-Software Co-Design featuring Intel Cyclone IV FPGA & ESP32 IoT Microcontroller*

---

## 📖 Overview

**Farmware-Bot** is an autonomous agricultural robotic rover engineered for crop maze navigation, precision farming, and soil/environmental monitoring. The system leverages a **heterogeneous computing architecture**:

1. **Deterministic Hardware Core (FPGA)**: An Intel Cyclone IV E (DE0-Nano) running custom Verilog HDL manages hard real-time tasks: sub-millimeter ultrasonic echo timing, a multi-state obstacle avoidance and maze navigation FSM, 6-bit PWM motor driving with per-motor speed trim, and full-duplex UART communication.
2. **IoT & Telemetry Layer (ESP32)**: Hosts a standalone Wi-Fi Access Point and embedded Web Server providing an interactive remote control dashboard, while orchestrating environmental telemetry: motorized soil moisture probe insertion, ambient light intensity, and UV index monitoring.

---

## 🏛️ System Architecture

```
                      +-----------------------------+
                      |       User Smartphone       |
                      |       or Web Browser        |
                      +--------------+--------------+
                                     |
                         Wi-Fi AP ("MazeRover")
                                     |
                      +--------------v--------------+
                      |         ESP32 Core          |
                      |   - SoftAP & Web Dashboard  |
                      |   - BH1750 Light (I2C)      |
                      |   - S12SD UV Sensor (ADC)   |
                      |   - Soil Probe + Servo Arm  |
                      +--------------+--------------+
                                     |
                       UART (9600 Baud, 8N1, 3.3V)
                                     |
                      +--------------v--------------+
                      | Intel Cyclone IV (DE0-Nano) |
                      |   - UART RX/TX Modules      |
                      |   - 3x HC-SR04 Ultrasonic   |
                      |   - HW-870 IR Maze End Det. |
                      |   - Navigation FSM          |
                      |   - Dual 6-bit PWM Gen      |
                      |   - Motor Driver (TB6612FNG)|
                      +--------------+--------------+
                                     |
                      +--------------v--------------+
                      |     2x Geared DC Motors     |
                      |     TB6612FNG Dual H-Bridge |
                      +-----------------------------+
```

---

## 🌟 Key Features

- **Hard Real-Time Navigation & Obstacle Avoidance**:
  - 3x **HC-SR04 Ultrasonic Transceivers** (Left, Front, Right) with cycle-accurate pulse-width time-of-flight measurement yielding millimeter-precision distance data.
  - **HW-870 IR Optical Sensor** detects the maze destination / harvest zone.
  - Robust 10-state finite state machine (FSM) navigating narrow corridors and dead ends (`FORWARD`, `STOP`, `CHECK`, `TURN_90`, `TURN_180`, `WAIT`, `RESUME`, `MAZE_DONE`, `IDLE`).
- **Precision Motor Drive**:
  - Independent 6-bit PWM speed control (period = 64 cycles, ~1.56% resolution) per motor channel with speed trimming to counteract mechanical friction discrepancies.
  - **TB6612FNG** dual MOSFET H-bridge driver delivering high efficiency and fast switching.
- **Smart Agricultural Sensing Payload**:
  - **Soil Moisture Sensor**: Mounted to a motorized SG90 servo arm (`SERVO_DOWN` = 150°, `SERVO_UP` = 60°). The probe lowers into the soil on command and retracts prior to movement to prevent damage.
  - **BH1750 Ambient Light Sensor**: Measures crop canopy sunlight exposure in Lux over I2C.
  - **S12SD UV Sensor**: Monitors ambient UV radiation index via high-precision 12-bit ADC.
- **Wireless Web Dashboard (No App Required)**:
  - ESP32 serves a responsive web dashboard over Wi-Fi (`http://192.168.4.1`).
  - Remote triggers: `START`, `STOP`, `RETURN TO START`, `READ LIGHT`, `READ UV`, and `TEST SOIL`.
  - Live status telemetry and battery/sensor monitoring.

---

## 🔌 Hardware Pinouts & Wiring

### 1. Intel Cyclone IV FPGA (DE0-Nano / EP4CE22F17C6)

| Signal | FPGA Pin | Description |
| :--- | :--- | :--- |
| `CLOCK_50` | `PIN_R8` | Onboard 50 MHz Master Clock |
| `KEY[0]` | `PIN_J15` | Active-LOW Reset / Button 0 |
| `KEY[1]` | `PIN_E1` | Auxiliary Button 1 |
| `LED[7:0]` | `PIN_L3`, `B1`, `F3`, `D1`, `A11`, `B13`, `A13`, `A15` | Status LEDs (State & Diagnostics) |
| `echo_left` | `PIN_A8` | HC-SR04 Left Echo (Dedicated Input) |
| `echo_front` | `PIN_B8` | HC-SR04 Front Echo (Dedicated Input) |
| `echo_right` | `PIN_A2` | HC-SR04 Right Echo (Dedicated Input) |
| `trig_left` | `PIN_D3` | HC-SR04 Left Trigger Pulse |
| `trig_front` | `PIN_C3` | HC-SR04 Front Trigger Pulse |
| `trig_right` | `PIN_A3` | HC-SR04 Right Trigger Pulse |
| `motor_AIN1` | `PIN_B3` | TB6612FNG Channel A IN1 (Left Motor) |
| `motor_AIN2` | `PIN_B4` | TB6612FNG Channel A IN2 (Left Motor) |
| `motor_PWMA` | `PIN_A4` | TB6612FNG Channel A PWM |
| `motor_BIN1` | `PIN_B5` | TB6612FNG Channel B IN1 (Right Motor) |
| `motor_BIN2` | `PIN_A5` | TB6612FNG Channel B IN2 (Right Motor) |
| `motor_PWMB` | `PIN_D5` | TB6612FNG Channel B PWM |
| `motor_STBY` | `PIN_B6` | TB6612FNG Standby Enable (Active HIGH) |
| `ir_maze_end` | `PIN_A6` | HW-870 IR Sensor Output |
| `uart_tx_out` | `PIN_A7` | FPGA UART TX &rarr; ESP32 RX (GPIO 3 / 16) |
| `uart_rx_in` | `PIN_B7` | FPGA UART RX &larr; ESP32 TX (GPIO 1 / 17) |

### 2. ESP32 Microcontroller

| Peripheral | ESP32 Pin | Function |
| :--- | :--- | :--- |
| `UART TX / RX` | `GPIO 1` (TX), `GPIO 3` (RX) | Bidirectional link to FPGA (`uart_rx_in`, `uart_tx_out`) |
| `Servo Motor` | `GPIO 18` | Soil Probe Deployment Actuator |
| `UV Sensor` | `GPIO 34` | S12SD Analog Output (ADC1 Channel 6) |
| `Soil Sensor` | `GPIO 35` | Soil Moisture Analog Output (ADC1 Channel 7) |
| `I2C SDA` | `GPIO 21` | BH1750 Ambient Light Sensor SDA |
| `I2C SCL` | `GPIO 22` | BH1750 Ambient Light Sensor SCL |

---

## 📡 Communication Protocol (UART 9600 Baud, 8N1)

### ESP32 &rarr; FPGA Commands
| Byte | Hex Code | Action |
| :--- | :--- | :--- |
| `CMD_START` | `0x01` | Transitions FSM from `S_IDLE` to `S_FORWARD` to initiate traversal |
| `CMD_STOP` | `0x02` | Immediate safe stop; forces FSM into `S_IDLE` |
| `CMD_RETURN` | `0x03` | Executes 180° turn from destination and resumes navigation to start |

### FPGA &rarr; ESP32 Status
| Byte | Hex Code | Status |
| :--- | :--- | :--- |
| `STATUS_MAZE_DONE` | `0xD1` / `0xAA` | Destination / maze end reached (IR detected) |
| `STATUS_RUNNING` | `0xD2` / `0xBB` | Autonomous navigation active |
| `STATUS_IDLE` | `0xD3` / `0xCC` | Motors stopped, waiting for user command |

---

## 📁 Repository Structure

```
Farmware-Bot/
├── .gitignore                   # Comprehensive ignore rules for Quartus, ESP32, OS
├── README.md                    # Project documentation & technical specifications
├── object_avoiding_robo.qpf     # Intel Quartus Prime Project File
├── object_avoiding_robo.qsf     # Quartus Settings & Pin Assignments
│
├── rtl/                         # Verilog HDL Register-Transfer Level Sources
│   ├── robo_top_level.v         # Top-level module interconnecting all subsystems
│   ├── motor_controller.v       # 10-state navigation & avoidance FSM
│   ├── ultrasonic.v             # Ultrasonic trigger generator & echo pulse timer
│   ├── pwd_generator.v          # 6-bit non-blocking PWM generator
│   ├── clk_divider.v            # Parameterizable clock frequency divider
│   ├── encoder.v                # Dual-channel quadrature encoder decoder
│   ├── uart_tx.v                # Hardware UART Transmitter (50MHz -> 9600 baud)
│   ├── uart_rx.v                # Hardware UART Receiver with 2-FF synchronizer
│   └── tb.v                     # RTL simulation test module
│
├── esp32_maze_rover/            # ESP32 Firmware
│   └── esp32_maze_rover.ino     # Wi-Fi SoftAP, Web Server, Sensor & UART Controller
│
└── tb/                          # Verification Testbenches
    └── tb_test.v                # Comprehensive testbench for simulation
```

---

## 🚀 Getting Started

### Prerequisites

- **FPGA Synthesis**: Intel Quartus Prime (Lite / Standard Edition &ge; 20.1)
- **Microcontroller**: Arduino IDE (or PlatformIO / ESP-IDF) with ESP32 board support
- **Arduino Libraries**:
  - `ESP32Servo`
  - `BH1750`
  - Built-in: `WiFi`, `WebServer`, `Wire`

---

### Step 1: FPGA Bitstream Synthesis & Flashing

1. Launch **Quartus Prime**.
2. Open [`object_avoiding_robo.qpf`](file:///c:/Users/BHAVIK/Documents/Bhavik/Projects/vlsi_projects/maze_solver/Farmware-Bot_git/object_avoiding_robo.qpf).
3. Connect your DE0-Nano board via USB-Blaster.
4. Run **Processing &rarr; Start Compilation** (`Ctrl+L`).
5. Open **Tools &rarr; Programmer**, select the compiled `.sof` file in `output_files/`, and click **Start** to program the Cyclone IV FPGA.

### Step 2: ESP32 Firmware Upload

1. Open [`esp32_maze_rover/esp32_maze_rover.ino`](file:///c:/Users/BHAVIK/Documents/Bhavik/Projects/vlsi_projects/maze_solver/Farmware-Bot_git/esp32_maze_rover/esp32_maze_rover.ino) in the Arduino IDE.
2. Select your ESP32 board (e.g., *DOIT ESP32 DEVKIT V1* or *ESP32 Dev Module*).
3. Ensure required libraries (`ESP32Servo`, `BH1750`) are installed via the Library Manager.
4. Compile and flash the sketch to your ESP32 board over USB.

### Step 3: Operating the Rover

1. Power the rover power rails (ensure motor ground, FPGA ground, and ESP32 ground are common).
2. On your smartphone or laptop, scan for Wi-Fi networks and connect to:
   - **SSID**: `MazeRover`
   - **Password**: `rover1234`
3. Open a browser and navigate to:
   ```
   http://192.168.4.1
   ```
4. Use the web dashboard to start/stop the rover and trigger environmental soil and light readings!

---

## 🛡️ License

Developed for precision agriculture and hardware-accelerated robotics research.
