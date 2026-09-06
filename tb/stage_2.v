`timescale 1ns/1ps

// =============================================================================
// TESTBENCH - STAGE 2: Autonomous Maze Solving Rover (DE0-Nano + ESP32)
// Full-duplex UART, TB6612FNG Dual H-Bridge, HW-870 IR Sensor, and 10-state FSM
// =============================================================================
module tb_stage_2;

// ---- Clocks & Resets --------------------------------------------------------
reg        clk;
reg [1:0]  KEY;

// ---- HC-SR04 Ultrasonic Sensors ---------------------------------------------
wire       trig_left,  trig_front,  trig_right;
reg        echo_left_tb, echo_front_tb, echo_right_tb;
reg [31:0] front_echo_cycles;
reg [31:0] left_echo_cycles;
reg [31:0] right_echo_cycles;

// ---- TB6612FNG Motor Driver -------------------------------------------------
wire       motor_AIN1, motor_AIN2, motor_PWMA;
wire       motor_BIN1, motor_BIN2, motor_PWMB;
wire       motor_STBY;

// ---- HW-870 IR Sensor -------------------------------------------------------
reg        ir_maze_end_tb;

// ---- UART Interface (ESP32 <-> FPGA) ----------------------------------------
reg        uart_rx_in_tb;  // Testbench drives FPGA RX (ESP32 TX)
wire       uart_tx_out;    // FPGA TX monitored by Testbench (ESP32 RX)

// ---- LEDs -------------------------------------------------------------------
wire [7:0] LED;

// ---- Test Tracking ----------------------------------------------------------
integer pass_count;
integer fail_count;

// =============================================================================
// CLOCK GENERATOR: 50 MHz (20ns period)
// =============================================================================
initial clk = 1'b0;
always  #10 clk = ~clk;

// =============================================================================
// TIMING CONSTANTS (Simulation-Optimized)
// =============================================================================
localparam [31:0] SIM_TURN_90_CYCLES   = 32'd20_000;   // 400us @ 50MHz
localparam [31:0] SIM_TURN_180_CYCLES  = 32'd40_000;   // 800us @ 50MHz
localparam [31:0] SIM_STOP_WAIT_CYCLES = 32'd10_000;   // 200us @ 50MHz
localparam [31:0] SIM_SENSOR_WARMUP    = 32'd20_000;   // 400us @ 50MHz

localparam [15:0] THRESHOLD_FRONT      = 16'd80;       // 80mm
localparam [15:0] THRESHOLD_SIDE       = 16'd80;       // 80mm

// Ultrasonic pulse widths:
// Distance mm = (echo_cycles * 339) / 100000
// 80mm threshold <=> ~23,598 cycles
localparam [31:0] ECHO_CLEAR    = 32'd60_000;   // ~203 mm (Clear)
localparam [31:0] ECHO_OBSTACLE = 32'd12_000;   // ~40 mm  (Obstacle detected)

// UART Baud Timing: 9600 baud @ 50 MHz = 5208 cycles/bit
localparam [31:0] UART_BIT_CYCLES = 32'd5208;

// Command bytes (ESP32 -> FPGA)
localparam [7:0] CMD_START  = 8'h01;
localparam [7:0] CMD_STOP   = 8'h02;
localparam [7:0] CMD_RETURN = 8'h03;

// Status bytes (FPGA -> ESP32)
localparam [7:0] STATUS_MAZE_DONE = 8'hD1;
localparam [7:0] STATUS_RUNNING   = 8'hD2;
localparam [7:0] STATUS_IDLE      = 8'hD3;

// =============================================================================
// DUT INSTANTIATION
// =============================================================================
robo_top_level #(
    .FRONT_THRESHOLD  (THRESHOLD_FRONT),
    .SIDE_THRESHOLD   (THRESHOLD_SIDE),
    .TURN_90_CYCLES   (SIM_TURN_90_CYCLES),
    .TURN_180_CYCLES  (SIM_TURN_180_CYCLES),
    .STOP_WAIT_CYCLES (SIM_STOP_WAIT_CYCLES),
    .SENSOR_WARMUP    (SIM_SENSOR_WARMUP)
) dut (
    .CLOCK_50        (clk),
    .KEY             (KEY),
    .LED             (LED),
    .trig_left       (trig_left),
    .echo_left       (echo_left_tb),
    .trig_front      (trig_front),
    .echo_front      (echo_front_tb),
    .trig_right      (trig_right),
    .echo_right      (echo_right_tb),
    .motor_AIN1      (motor_AIN1),
    .motor_AIN2      (motor_AIN2),
    .motor_PWMA      (motor_PWMA),
    .motor_BIN1      (motor_BIN1),
    .motor_BIN2      (motor_BIN2),
    .motor_PWMB      (motor_PWMB),
    .motor_STBY      (motor_STBY),
    .ir_maze_end     (ir_maze_end_tb),
    .uart_tx_out     (uart_tx_out),
    .uart_rx_in      (uart_rx_in_tb)
);

// =============================================================================
// CONCURRENT HC-SR04 ECHO RESPONDERS
// =============================================================================
always @(posedge trig_front) begin
    repeat(10) @(posedge clk);
    echo_front_tb = 1'b1;
    repeat(front_echo_cycles) @(posedge clk);
    echo_front_tb = 1'b0;
end

always @(posedge trig_left) begin
    repeat(10) @(posedge clk);
    echo_left_tb = 1'b1;
    repeat(left_echo_cycles) @(posedge clk);
    echo_left_tb = 1'b0;
end

always @(posedge trig_right) begin
    repeat(10) @(posedge clk);
    echo_right_tb = 1'b1;
    repeat(right_echo_cycles) @(posedge clk);
    echo_right_tb = 1'b0;
end

// =============================================================================
// UART TELEMETRY MONITOR (FPGA TX -> ESP32 RX)
// =============================================================================
reg [7:0]  mon_rx_byte;
reg        mon_rx_valid;
integer    bit_cnt;

initial begin
    mon_rx_byte  = 8'h00;
    mon_rx_valid = 1'b0;
    forever begin
        mon_rx_valid = 1'b0;
        // Wait for start bit falling edge
        @(negedge uart_tx_out);
        // Sample at mid-point of start bit
        repeat(UART_BIT_CYCLES / 2) @(posedge clk);
        if (uart_tx_out === 1'b0) begin
            // Valid start bit, sample 8 data bits
            for (bit_cnt = 0; bit_cnt < 8; bit_cnt = bit_cnt + 1) begin
                repeat(UART_BIT_CYCLES) @(posedge clk);
                mon_rx_byte[bit_cnt] = uart_tx_out;
            end
            // Sample stop bit
            repeat(UART_BIT_CYCLES) @(posedge clk);
            if (uart_tx_out === 1'b1) begin
                mon_rx_valid = 1'b1;
                $display("[%0t ns] [UART TELEMETRY RCVD] 0x%02X (%s)",
                         $time, mon_rx_byte,
                         (mon_rx_byte == STATUS_RUNNING)   ? "STATUS_RUNNING" :
                         (mon_rx_byte == STATUS_MAZE_DONE) ? "STATUS_MAZE_DONE" :
                         (mon_rx_byte == STATUS_IDLE)      ? "STATUS_IDLE" : "HEARTBEAT/DATA");
            end else begin
                // If line remains 0, it is maze_done hardware override
                $display("[%0t ns] [UART TX] Line held LOW (Maze Done Override or Framing)", $time);
                @(posedge uart_tx_out); // Wait for line to return high before listening again
            end
        end
    end
end

// =============================================================================
// FSM STATE MONITOR
// =============================================================================
reg [3:0] prev_state;
initial   prev_state = 4'hF;

always @(posedge clk) begin
    if (LED[3:0] !== prev_state) begin
        prev_state = LED[3:0];
        case (LED[3:0])
            4'd0: $display("[%0t ns] >> FSM STATE -> S_FORWARD (0)",    $time);
            4'd1: $display("[%0t ns] >> FSM STATE -> S_STOP (1)",       $time);
            4'd2: $display("[%0t ns] >> FSM STATE -> S_CHECK (2)",      $time);
            4'd3: $display("[%0t ns] >> FSM STATE -> S_TURN_RIGHT (3)", $time);
            4'd4: $display("[%0t ns] >> FSM STATE -> S_TURN_LEFT (4)",  $time);
            4'd5: $display("[%0t ns] >> FSM STATE -> S_TURN_180 (5)",   $time);
            4'd6: $display("[%0t ns] >> FSM STATE -> S_WAIT_180 (6)",   $time);
            4'd7: $display("[%0t ns] >> FSM STATE -> S_RESUME (7)",     $time);
            4'd8: $display("[%0t ns] >> FSM STATE -> S_IDLE (8)",       $time);
            4'd9: $display("[%0t ns] >> FSM STATE -> S_MAZE_DONE (9)",  $time);
            default: $display("[%0t ns] >> FSM STATE -> UNKNOWN (%0d)", $time, LED[3:0]);
        endcase
    end
end

// =============================================================================
// HELPER TASKS
// =============================================================================
task wait_cycles;
    input [31:0] n;
    reg   [31:0] i;
    begin
        for (i = 0; i < n; i = i + 1)
            @(posedge clk);
    end
endtask

// Wait for a full ultrasonic measurement cycle to complete and update distance registers
task wait_for_sensor_cycle;
    begin
        @(posedge trig_front);
        repeat(100_000) @(posedge clk);
    end
endtask

task wait_for_state;
    input [3:0]  expected;
    input [31:0] timeout;
    reg   [31:0] t;
    begin
        t = 0;
        while (LED[3:0] !== expected && t < timeout) begin
            @(posedge clk);
            t = t + 1;
        end
        if (LED[3:0] === expected) begin
            $display("[%0t ns] PASS: State %0d reached in %0d cycles", $time, expected, t);
            pass_count = pass_count + 1;
        end else begin
            $display("[%0t ns] FAIL: Timeout waiting for state %0d (current=%0d)",
                     $time, expected, LED[3:0]);
            fail_count = fail_count + 1;
        end
    end
endtask

task send_uart_byte;
    input [7:0] byte_val;
    integer i;
    begin
        $display("[%0t ns] [UART TX TO FPGA] Sending byte 0x%02X...", $time, byte_val);
        // Start bit (active low)
        uart_rx_in_tb = 1'b0;
        repeat(UART_BIT_CYCLES) @(posedge clk);

        // 8 Data bits (LSB first)
        for (i = 0; i < 8; i = i + 1) begin
            uart_rx_in_tb = byte_val[i];
            repeat(UART_BIT_CYCLES) @(posedge clk);
        end

        // Stop bit (active high)
        uart_rx_in_tb = 1'b1;
        repeat(UART_BIT_CYCLES) @(posedge clk);
        $display("[%0t ns] [UART TX TO FPGA] Finished sending byte 0x%02X", $time, byte_val);
    end
endtask

// =============================================================================
// MAIN TEST SUITE
// =============================================================================
initial begin
    $display("==================================================================");
    $display("===  FARMWARE-BOT / MAZE ROVER STAGE 2 TESTBENCH START         ===");
    $display("==================================================================");

    pass_count = 0;
    fail_count = 0;

    // Default pin drives
    KEY               = 2'b11;    // KEY[0]=Reset(H), KEY[1]=ManualStart(H)
    echo_front_tb     = 1'b0;
    echo_left_tb      = 1'b0;
    echo_right_tb     = 1'b0;
    front_echo_cycles = ECHO_CLEAR;
    left_echo_cycles  = ECHO_CLEAR;
    right_echo_cycles = ECHO_CLEAR;
    ir_maze_end_tb    = 1'b0;
    uart_rx_in_tb     = 1'b1;     // UART idle high

    // -------------------------------------------------------------------------
    // TEST 1: Power-on & Reset -> Verify S_IDLE (8) & Motors OFF
    // -------------------------------------------------------------------------
    $display("\n--- TEST 1: Power-on Reset & Verify S_IDLE ---");
    KEY[0] = 1'b0;   // Assert active-low reset
    wait_cycles(32'd200);
    KEY[0] = 1'b1;   // Release reset
    wait_cycles(32'd500);

    if (LED[3:0] === 4'd8 &&
        motor_AIN1 === 1'b0 && motor_AIN2 === 1'b0 &&
        motor_BIN1 === 1'b0 && motor_BIN2 === 1'b0 &&
        motor_STBY === 1'b1 && LED[7] === 1'b0) begin
        $display("[%0t ns] PASS: Rover initialized in S_IDLE (8), motors off, STBY high", $time);
        pass_count = pass_count + 1;
    end else begin
        $display("[%0t ns] FAIL: S_IDLE check failed! LED[3:0]=%d, AIN1=%b, BIN1=%b, STBY=%b",
                 $time, LED[3:0], motor_AIN1, motor_BIN1, motor_STBY);
        fail_count = fail_count + 1;
    end

    // Wait for ultrasonic warmup guard and first clean clear reading
    $display("[%0t ns] Waiting for sensor warmup (%0d cycles)...", $time, SIM_SENSOR_WARMUP);
    wait_cycles(SIM_SENSOR_WARMUP + 32'd5_000);
    wait_for_sensor_cycle;

    // -------------------------------------------------------------------------
    // TEST 2: UART START Command (0x01) -> Transition to S_FORWARD (0)
    // -------------------------------------------------------------------------
    $display("\n--- TEST 2: UART CMD_START (0x01) -> S_FORWARD ---");
    send_uart_byte(CMD_START);

    wait_for_state(4'd0, 32'd20_000);  // S_FORWARD
    wait_cycles(32'd1_000);

    // Check motor forward direction: AIN1=1, AIN2=0, BIN1=1, BIN2=0
    if (motor_AIN1 === 1'b1 && motor_AIN2 === 1'b0 &&
        motor_BIN1 === 1'b1 && motor_BIN2 === 1'b0) begin
        $display("[%0t ns] PASS: Motor direction pins correct for FORWARD (AIN1=1, BIN1=1)", $time);
        pass_count = pass_count + 1;
    end else begin
        $display("[%0t ns] FAIL: Motor direction incorrect in S_FORWARD! AIN=(%b,%b), BIN=(%b,%b)",
                 $time, motor_AIN1, motor_AIN2, motor_BIN1, motor_BIN2);
        fail_count = fail_count + 1;
    end

    // -------------------------------------------------------------------------
    // TEST 3: Front Obstacle, Sides Clear -> Turn Right Sequence
    // -------------------------------------------------------------------------
    $display("\n--- TEST 3: Obstacle Avoidance: Front blocked, sides clear -> TURN_RIGHT ---");
    front_echo_cycles = ECHO_OBSTACLE;
    left_echo_cycles  = ECHO_CLEAR;
    right_echo_cycles = ECHO_CLEAR;

    wait_for_state(4'd1, 32'd3_500_000); // S_STOP (wait for next ultrasonic cycle)
    wait_for_state(4'd2, 32'd50_000);    // S_CHECK
    wait_for_state(4'd3, 32'd50_000);    // S_TURN_RIGHT

    // Check motor spin right: Left fwd (AIN1=1, AIN2=0), Right bwd (BIN1=0, BIN2=1)
    if (motor_AIN1 === 1'b1 && motor_AIN2 === 1'b0 &&
        motor_BIN1 === 1'b0 && motor_BIN2 === 1'b1) begin
        $display("[%0t ns] PASS: Motors spinning clockwise (Right Turn: AIN1=1, BIN2=1)", $time);
        pass_count = pass_count + 1;
    end else begin
        $display("[%0t ns] FAIL: Motor turn right failed! AIN=(%b,%b), BIN=(%b,%b)",
                 $time, motor_AIN1, motor_AIN2, motor_BIN1, motor_BIN2);
        fail_count = fail_count + 1;
    end

    wait_for_state(4'd7, SIM_TURN_90_CYCLES + 32'd10_000); // S_RESUME

    // While turning/resuming, clear front so when it returns to FORWARD it stays clear
    front_echo_cycles = ECHO_CLEAR;
    wait_for_state(4'd0, SIM_STOP_WAIT_CYCLES + 32'd10_000); // Back to S_FORWARD

    // Wait for the next sensor cycle to latch CLEAR distance before next test
    wait_for_sensor_cycle;

    // -------------------------------------------------------------------------
    // TEST 4: Front + Right Obstacle, Left Clear -> Turn Left Sequence
    // -------------------------------------------------------------------------
    $display("\n--- TEST 4: Obstacle Avoidance: Front & Right blocked -> TURN_LEFT ---");
    front_echo_cycles = ECHO_OBSTACLE;
    right_echo_cycles = ECHO_OBSTACLE;
    left_echo_cycles  = ECHO_CLEAR;

    wait_for_state(4'd1, 32'd3_500_000); // S_STOP
    wait_for_state(4'd2, 32'd50_000);    // S_CHECK
    wait_for_state(4'd4, 32'd50_000);    // S_TURN_LEFT

    // Check motor spin left: Left bwd (AIN1=0, AIN2=1), Right fwd (BIN1=1, BIN2=0)
    if (motor_AIN1 === 1'b0 && motor_AIN2 === 1'b1 &&
        motor_BIN1 === 1'b1 && motor_BIN2 === 1'b0) begin
        $display("[%0t ns] PASS: Motors spinning counter-clockwise (Left Turn: AIN2=1, BIN1=1)", $time);
        pass_count = pass_count + 1;
    end else begin
        $display("[%0t ns] FAIL: Motor turn left failed! AIN=(%b,%b), BIN=(%b,%b)",
                 $time, motor_AIN1, motor_AIN2, motor_BIN1, motor_BIN2);
        fail_count = fail_count + 1;
    end

    wait_for_state(4'd7, SIM_TURN_90_CYCLES + 32'd10_000); // S_RESUME

    // Clear echoes
    front_echo_cycles = ECHO_CLEAR;
    right_echo_cycles = ECHO_CLEAR;
    wait_for_state(4'd0, SIM_STOP_WAIT_CYCLES + 32'd10_000); // S_FORWARD

    wait_for_sensor_cycle;

    // -------------------------------------------------------------------------
    // TEST 5: Dead End (Front, Left, Right Blocked) -> Turn 180 Sequence
    // -------------------------------------------------------------------------
    $display("\n--- TEST 5: Obstacle Avoidance: Dead End (All 3 blocked) -> TURN_180 ---");
    front_echo_cycles = ECHO_OBSTACLE;
    left_echo_cycles  = ECHO_OBSTACLE;
    right_echo_cycles = ECHO_OBSTACLE;

    wait_for_state(4'd1, 32'd3_500_000); // S_STOP
    wait_for_state(4'd2, 32'd50_000);    // S_CHECK
    wait_for_state(4'd6, 32'd50_000);    // S_WAIT_180
    wait_for_state(4'd5, SIM_STOP_WAIT_CYCLES + 32'd10_000); // S_TURN_180

    // Spin 180 (same spin as TURN_RIGHT)
    if (motor_AIN1 === 1'b1 && motor_AIN2 === 1'b0 &&
        motor_BIN1 === 1'b0 && motor_BIN2 === 1'b1) begin
        $display("[%0t ns] PASS: Motors spinning 180 degrees (AIN1=1, BIN2=1)", $time);
        pass_count = pass_count + 1;
    end else begin
        $display("[%0t ns] FAIL: Motor 180 spin failed!", $time);
        fail_count = fail_count + 1;
    end

    wait_for_state(4'd7, SIM_TURN_180_CYCLES + 32'd10_000); // S_RESUME

    front_echo_cycles = ECHO_CLEAR;
    left_echo_cycles  = ECHO_CLEAR;
    right_echo_cycles = ECHO_CLEAR;
    wait_for_state(4'd0, SIM_STOP_WAIT_CYCLES + 32'd10_000); // S_FORWARD

    wait_for_sensor_cycle;

    // -------------------------------------------------------------------------
    // TEST 6: HW-870 IR Sensor Detection -> S_MAZE_DONE (9) & TX Override
    // -------------------------------------------------------------------------
    $display("\n--- TEST 6: HW-870 IR Maze End Detection -> S_MAZE_DONE ---");
    ir_maze_end_tb = 1'b1;  // Black surface / maze end detected
    wait_cycles(32'd1_000); // Wait through debounce (50 cycles)

    wait_for_state(4'd9, 32'd10_000); // S_MAZE_DONE

    // In S_MAZE_DONE: motors halt, LED[7]=1, LED[5]=1, uart_tx_out held LOW
    if (motor_AIN1 === 1'b0 && motor_AIN2 === 1'b0 &&
        motor_BIN1 === 1'b0 && motor_BIN2 === 1'b0 &&
        LED[7] === 1'b1 && LED[5] === 1'b1 && uart_tx_out === 1'b0) begin
        $display("[%0t ns] PASS: S_MAZE_DONE verified: Motors off, LED[7]=1, LED[5]=1, UART_TX held LOW", $time);
        pass_count = pass_count + 1;
    end else begin
        $display("[%0t ns] FAIL: S_MAZE_DONE outputs incorrect! LED[7]=%b, LED[5]=%b, TX=%b",
                 $time, LED[7], LED[5], uart_tx_out);
        fail_count = fail_count + 1;
    end

    // -------------------------------------------------------------------------
    // TEST 7: UART RETURN Command (0x03) -> 180 Turn and Resume Navigation
    // -------------------------------------------------------------------------
    $display("\n--- TEST 7: UART CMD_RETURN (0x03) from S_MAZE_DONE ---");
    ir_maze_end_tb = 1'b0;  // Cleared IR marker
    send_uart_byte(CMD_RETURN);

    wait_for_state(4'd6, 32'd20_000);                         // S_WAIT_180
    wait_for_state(4'd5, SIM_STOP_WAIT_CYCLES + 32'd10_000);  // S_TURN_180
    wait_for_state(4'd7, SIM_TURN_180_CYCLES + 32'd10_000);  // S_RESUME
    wait_for_state(4'd0, SIM_STOP_WAIT_CYCLES + 32'd10_000);  // S_FORWARD

    if (LED[7] === 1'b0 && motor_AIN1 === 1'b1 && motor_BIN1 === 1'b1) begin
        $display("[%0t ns] PASS: Return sequence complete: Maze Done cleared, Rover driving FORWARD", $time);
        pass_count = pass_count + 1;
    end else begin
        $display("[%0t ns] FAIL: Return sequence failed! LED[7]=%b, AIN1=%b, BIN1=%b",
                 $time, LED[7], motor_AIN1, motor_BIN1);
        fail_count = fail_count + 1;
    end

    // -------------------------------------------------------------------------
    // TEST 8: UART STOP Command (0x02) -> Immediate Halt to S_IDLE (8)
    // -------------------------------------------------------------------------
    $display("\n--- TEST 8: UART CMD_STOP (0x02) Emergency Halt ---");
    send_uart_byte(CMD_STOP);

    wait_for_state(4'd8, 32'd20_000);  // S_IDLE
    wait_cycles(32'd500);

    if (motor_AIN1 === 1'b0 && motor_BIN1 === 1'b0) begin
        $display("[%0t ns] PASS: Emergency STOP halted motors to S_IDLE", $time);
        pass_count = pass_count + 1;
    end else begin
        $display("[%0t ns] FAIL: Motors still active after STOP! AIN1=%b, BIN1=%b",
                 $time, motor_AIN1, motor_BIN1);
        fail_count = fail_count + 1;
    end

    // -------------------------------------------------------------------------
    // TEST 9: Pushbutton KEY[1] Manual Start -> S_FORWARD, then STOP
    // -------------------------------------------------------------------------
    $display("\n--- TEST 9: Manual Start Button KEY[1] ---");
    // Simulate active-low button press
    KEY[1] = 1'b0;
    wait_cycles(32'd200);
    KEY[1] = 1'b1;

    wait_for_state(4'd0, 32'd20_000);  // S_FORWARD
    wait_cycles(32'd1_000);

    if (motor_AIN1 === 1'b1 && motor_BIN1 === 1'b1) begin
        $display("[%0t ns] PASS: Pushbutton KEY[1] successfully started rover", $time);
        pass_count = pass_count + 1;
    end else begin
        $display("[%0t ns] FAIL: KEY[1] start failed! AIN1=%b, BIN1=%b",
                 $time, motor_AIN1, motor_BIN1);
        fail_count = fail_count + 1;
    end

    // Final halt
    send_uart_byte(CMD_STOP);
    wait_for_state(4'd8, 32'd20_000);
    wait_cycles(32'd2_000);

    // =========================================================================
    // FINAL RESULTS SUMMARY
    // =========================================================================
    $display("\n==================================================================");
    $display("=== STAGE 2 TESTBENCH RESULTS: %0d PASSED, %0d FAILED          ===",
             pass_count, fail_count);
    $display("==================================================================");

    if (fail_count == 0)
        $display("\n>>> ALL TESTS PASSED SUCCESSFULLY! <<<\n");
    else
        $display("\n>>> SOME TESTS FAILED! <<<\n");

    $stop;
end

endmodule
