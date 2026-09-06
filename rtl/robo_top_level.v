// =============================================================================
// TOP LEVEL MODULE - Maze Solving Rover
// DE0-Nano FPGA + 3x HC-SR04 + 2x DC 300RPM Geared Motor + TB6612FNG
// + HW-870 IR Sensor (maze end) + UART <-> ESP32
//
// UART Protocol (9600 baud, 8N1):
//   ESP32 -> FPGA:  0x01=START, 0x02=STOP, 0x03=RETURN
//   FPGA -> ESP32:  0xAA=MAZE_DONE, 0xBB=RUNNING, 0xCC=IDLE
// =============================================================================
module robo_top_level #(
    parameter FRONT_THRESHOLD  = 16'd80,    // 8 cm
    parameter SIDE_THRESHOLD   = 16'd80,    // 8 cm

    // Hardware timing for 300RPM Geared DC Motor @ 50MHz (override in simulation)
    parameter TURN_90_CYCLES   = 32'd55_000_000,
    parameter TURN_180_CYCLES  = 32'd110_000_000,
    parameter STOP_WAIT_CYCLES = 32'd50_000_000,
    parameter SENSOR_WARMUP    = 32'd1_500_000
)(
    input  wire        CLOCK_50,
    input  wire [1:0]  KEY,
    output wire [7:0]  LED,

    // HC-SR04 Ultrasonic sensors
    output wire        trig_left,
    input  wire        echo_left,
    output wire        trig_front,
    input  wire        echo_front,
    output wire        trig_right,
    input  wire        echo_right,

    // TB6612FNG - Left Motor (Channel A)
    output wire        motor_AIN1,
    output wire        motor_AIN2,
    output wire        motor_PWMA,

    // TB6612FNG - Right Motor (Channel B)
    output wire        motor_BIN1,
    output wire        motor_BIN2,
    output wire        motor_PWMB,

    // TB6612FNG - Standby
    output wire        motor_STBY,

    // HW-870 IR Sensor (maze end detection)
    input  wire        ir_maze_end,

    // UART to ESP32
    output wire        uart_tx_out,   // FPGA TX -> ESP32 RX (GPIO16)
    input  wire        uart_rx_in     // FPGA RX <- ESP32 TX (GPIO17)
);

// =============================================================================
// PARAMETERS
// =============================================================================

// Per-motor speed trim (6-bit PWM, period=64)
localparam SPEED_FULL_L     = 6'd12;
localparam SPEED_FULL_R     = 6'd11;
localparam SPEED_TURN_L     = 6'd12;
localparam SPEED_TURN_R     = 6'd11;

// UART command bytes
localparam CMD_START_BYTE   = 8'h01;
localparam CMD_STOP_BYTE    = 8'h02;
localparam CMD_RETURN_BYTE  = 8'h03;

// UART status bytes (bit0=1 for clean start-bit edge on noisy lines)
localparam STATUS_MAZE_DONE = 8'hD1;
localparam STATUS_RUNNING   = 8'hD2;
localparam STATUS_IDLE      = 8'hD3;

// =============================================================================
// INTERNAL SIGNALS
// =============================================================================
wire reset = KEY[0];   // Active LOW

wire clk_3125KHz;
wire clk_195KHz_left, clk_195KHz_right;

wire [15:0] dist_left, dist_front, dist_right;
wire        obj_left,  obj_front,  obj_right;

wire [5:0]  duty_left,      duty_right;
wire        dir_left_fwd,   dir_left_bwd;
wire        dir_right_fwd,  dir_right_bwd;

wire pwm_left, pwm_right;
wire uart_tx_wire;  // Intermediate UART TX before maze_done override

// Motor controller outputs
wire [3:0] mc_state;
wire       maze_done;

// UART signals
wire [7:0] rx_data;
wire       rx_valid;
reg  [7:0] tx_data;
reg        tx_start;
wire       tx_busy;

// Command pulses (one-cycle each)
reg cmd_start_pulse;
reg cmd_stop_pulse;
reg cmd_return_pulse;

// Pushbutton KEY[1] synchronizer for manual start
reg key1_sync1, key1_sync2, key1_prev;

// Status tracking for UART TX
reg [3:0] prev_state;
reg       status_pending;
reg [7:0] status_byte;

// =============================================================================
// CLOCK DIVIDER: 50MHz -> ~80kHz (for PWM base clock)
// =============================================================================
clk_divider #(.DIVIDE_BY(312)) u_clkdiv(
    .clk_in  (CLOCK_50),
    .reset   (reset),
    .clk_out (clk_3125KHz)
);

// =============================================================================
// HC-SR04 SENSORS
// =============================================================================
t1b_ultrasonic u_sensor_left(
    .clk_50M     (CLOCK_50),
    .reset       (reset),
    .echo_rx     (echo_left),
    .trig        (trig_left),
    .op          (obj_left),
    .distance_out(dist_left)
);

t1b_ultrasonic u_sensor_front(
    .clk_50M     (CLOCK_50),
    .reset       (reset),
    .echo_rx     (echo_front),
    .trig        (trig_front),
    .op          (obj_front),
    .distance_out(dist_front)
);

t1b_ultrasonic u_sensor_right(
    .clk_50M     (CLOCK_50),
    .reset       (reset),
    .echo_rx     (echo_right),
    .trig        (trig_right),
    .op          (obj_right),
    .distance_out(dist_right)
);

// =============================================================================
// UART TX (FPGA -> ESP32)
// =============================================================================
uart_tx u_uart_tx(
    .clk      (CLOCK_50),
    .reset    (reset),
    .tx_data  (tx_data),
    .tx_start (tx_start),
    .tx_out   (uart_tx_wire),
    .tx_busy  (tx_busy)
);

// GPIO-level override: when maze_done, hold TX line LOW so ESP32 can
// detect it via digitalRead(3) even if UART reception is broken
assign uart_tx_out = maze_done ? 1'b0 : uart_tx_wire;

// =============================================================================
// UART RX (ESP32 -> FPGA)
// =============================================================================
uart_rx u_uart_rx(
    .clk      (CLOCK_50),
    .reset    (reset),
    .rx_in    (uart_rx_in),
    .rx_data  (rx_data),
    .rx_valid (rx_valid)
);

// =============================================================================
// COMMAND DECODER — parse UART RX bytes into one-cycle command pulses
// =============================================================================
always @(posedge CLOCK_50 or negedge reset) begin
    if (!reset) begin
        cmd_start_pulse  <= 1'b0;
        cmd_stop_pulse   <= 1'b0;
        cmd_return_pulse <= 1'b0;
        key1_sync1       <= 1'b1;
        key1_sync2       <= 1'b1;
        key1_prev        <= 1'b1;
    end else begin
        // Default: pulses last only one cycle
        cmd_start_pulse  <= 1'b0;
        cmd_stop_pulse   <= 1'b0;
        cmd_return_pulse <= 1'b0;

        // Synchronize KEY[1] (active-low push button)
        key1_sync1 <= KEY[1];
        key1_sync2 <= key1_sync1;
        key1_prev  <= key1_sync2;

        // Falling edge on KEY[1] (button press) generates a start pulse
        if (key1_prev == 1'b1 && key1_sync2 == 1'b0) begin
            cmd_start_pulse <= 1'b1;
        end

        // UART commands from ESP32
        if (rx_valid) begin
            case (rx_data)
                CMD_START_BYTE:  cmd_start_pulse  <= 1'b1;
                CMD_STOP_BYTE:   cmd_stop_pulse   <= 1'b1;
                CMD_RETURN_BYTE: cmd_return_pulse <= 1'b1;
                default: ;  // Ignore unknown bytes
            endcase
        end
    end
end

// =============================================================================
// STATUS SENDER — send status byte via UART TX on state changes & periodic heartbeat
// =============================================================================
reg [25:0] heartbeat_cnt;
localparam HEARTBEAT_CYCLES = 26'd500_000; // 10ms @ 50MHz (100 times/sec broadcast)

always @(posedge CLOCK_50 or negedge reset) begin
    if (!reset) begin
        prev_state     <= 4'd8;   // S_IDLE
        status_pending <= 1'b0;
        status_byte    <= 8'd0;
        tx_data        <= 8'd0;
        tx_start       <= 1'b0;
        heartbeat_cnt  <= 26'd0;
    end else begin
        tx_start <= 1'b0;  // Default: no transmit

        // Periodic heartbeat timer (every 40ms) to ensure ESP32 stays synced
        if (heartbeat_cnt >= HEARTBEAT_CYCLES) begin
            heartbeat_cnt  <= 26'd0;
            status_pending <= 1'b1;
            case (mc_state)
                4'd9:    status_byte <= STATUS_MAZE_DONE; // 0xAA
                4'd8:    status_byte <= STATUS_IDLE;      // 0xCC
                default: status_byte <= STATUS_RUNNING;   // 0xBB
            endcase
        end else begin
            heartbeat_cnt <= heartbeat_cnt + 26'd1;
        end

        // Detect immediate state changes
        if (mc_state != prev_state) begin
            prev_state    <= mc_state;
            heartbeat_cnt <= 26'd0; // Reset heartbeat counter on transition

            // Map state to status byte
            case (mc_state)
                4'd9:    begin status_byte <= STATUS_MAZE_DONE; status_pending <= 1'b1; end // S_MAZE_DONE (0xAA)
                4'd0:    begin status_byte <= STATUS_RUNNING;   status_pending <= 1'b1; end // S_FORWARD   (0xBB)
                4'd8:    begin status_byte <= STATUS_IDLE;      status_pending <= 1'b1; end // S_IDLE      (0xCC)
                default: ;  // Don't send for intermediate turning states
            endcase
        end

        // Send pending status when UART TX is free
        if (status_pending && !tx_busy) begin
            tx_data        <= status_byte;
            tx_start       <= 1'b1;
            status_pending <= 1'b0;
        end
    end
end

// =============================================================================
// MOTOR CONTROLLER STATE MACHINE
// =============================================================================
motor_controller #(
    .FRONT_THRESHOLD (FRONT_THRESHOLD),
    .SIDE_THRESHOLD  (SIDE_THRESHOLD),
    .TURN_90_CYCLES  (TURN_90_CYCLES),
    .TURN_180_CYCLES (TURN_180_CYCLES),
    .STOP_WAIT_CYCLES(STOP_WAIT_CYCLES),
    .SPEED_FULL_L    (SPEED_FULL_L),
    .SPEED_FULL_R    (SPEED_FULL_R),
    .SPEED_TURN_L    (SPEED_TURN_L),
    .SPEED_TURN_R    (SPEED_TURN_R),
    .SENSOR_WARMUP   (SENSOR_WARMUP),
    .IR_ACTIVE_LOW   (0)
) u_motor_ctrl(
    .clk             (CLOCK_50),
    .reset           (reset),
    .dist_front      (dist_front),
    .dist_left       (dist_left),
    .dist_right      (dist_right),
    .ir_maze_end     (ir_maze_end),
    .cmd_start       (cmd_start_pulse),
    .cmd_stop        (cmd_stop_pulse),
    .cmd_return      (cmd_return_pulse),
    .duty_left       (duty_left),
    .duty_right      (duty_right),
    .dir_left_fwd    (dir_left_fwd),
    .dir_left_bwd    (dir_left_bwd),
    .dir_right_fwd   (dir_right_fwd),
    .dir_right_bwd   (dir_right_bwd),
    .state_out       (mc_state),
    .maze_done       (maze_done)
);

// =============================================================================
// PWM GENERATORS
// =============================================================================
pwm_generator u_pwm_left(
    .clk_3125KHz (clk_3125KHz),
    .duty_cycle  (duty_left),
    .clk_195KHz  (clk_195KHz_left),
    .pwm_signal  (pwm_left)
);

pwm_generator u_pwm_right(
    .clk_3125KHz (clk_3125KHz),
    .duty_cycle  (duty_right),
    .clk_195KHz  (clk_195KHz_right),
    .pwm_signal  (pwm_right)
);

// =============================================================================
// MOTOR OUTPUT ASSIGNMENTS — TB6612FNG
// =============================================================================
assign motor_AIN1 = dir_left_fwd;
assign motor_AIN2 = dir_left_bwd;
assign motor_PWMA = pwm_left;

assign motor_BIN1 = dir_right_fwd;
assign motor_BIN2 = dir_right_bwd;
assign motor_PWMB = pwm_right;

assign motor_STBY = 1'b1;

// =============================================================================
// DEBUG LEDs
//   [3:0] = state machine state (0-9)
//   [4]   = front obstacle
//   [5]   = live IR sensor input (PIN_F3) — ON when black surface detected!
//   [6]   = right obstacle
//   [7]   = maze_done (PIN_L3)
// =============================================================================
assign LED[3:0] = mc_state;
assign LED[4]   = (dist_front < FRONT_THRESHOLD);
assign LED[5]   = ir_maze_end;
assign LED[6]   = (dist_right < SIDE_THRESHOLD);
assign LED[7]   = maze_done;

endmodule
