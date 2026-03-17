`timescale 1ns/1ps

// =============================================================================
// TESTBENCH - Object Avoiding Car (DE0-Nano)
// Uses simulation-short timing in motor_controller via top-level parameters.
// This version assumes robo_top_level and motor_controller are already using:
//   TURN_90_CYCLES   = 750_000
//   TURN_180_CYCLES  = 1_500_000
//   STOP_WAIT_CYCLES = 250_000
// =============================================================================
module tb;

// ---- Clock / Reset ----------------------------------------------------------
reg        clk;
reg [1:0]  KEY;

// ---- Echo control (per sensor) ----------------------------------------------
reg [31:0] front_echo_cycles;
reg [31:0] left_echo_cycles;
reg [31:0] right_echo_cycles;

reg echo_left_tb;
reg echo_front_tb;
reg echo_right_tb;

// ---- Encoders ---------------------------------------------------------------
reg enc_la, enc_lb, enc_ra, enc_rb;

// ---- DUT outputs ------------------------------------------------------------
wire [7:0] LED;
wire       trig_left, trig_front, trig_right;
wire       motor_left_in1,  motor_left_in2,  motor_left_ena;
wire       motor_right_in3, motor_right_in4, motor_right_enb;

// 50 MHz clock
initial clk = 0;
always  #10 clk = ~clk;

// ---- Echo pulse widths (50 MHz) ---------------------------------------------
localparam [31:0] ECHO_500MM = 32'd147493;  // clear
localparam [31:0] ECHO_200MM = 32'd58997;   // front wall
localparam [31:0] ECHO_150MM = 32'd44248;   // side wall
localparam [31:0] ECHO_100MM = 32'd29499;   // tight wall

// ---- DUT --------------------------------------------------------------------
robo_top_level dut(
    .CLOCK_50        (clk),
    .KEY             (KEY),
    .LED             (LED),
    .trig_left       (trig_left),
    .echo_left       (echo_left_tb),
    .trig_front      (trig_front),
    .echo_front      (echo_front_tb),
    .trig_right      (trig_right),
    .echo_right      (echo_right_tb),
    .motor_left_in1  (motor_left_in1),
    .motor_left_in2  (motor_left_in2),
    .motor_left_ena  (motor_left_ena),
    .enc_left_a      (enc_la),
    .enc_left_b      (enc_lb),
    .motor_right_in3 (motor_right_in3),
    .motor_right_in4 (motor_right_in4),
    .motor_right_enb (motor_right_enb),
    .enc_right_a     (enc_ra),
    .enc_right_b     (enc_rb)
);

// ---- Concurrent echo responders ---------------------------------------------
always @(posedge trig_front) begin
    repeat(5) @(posedge clk);
    echo_front_tb = 1'b1;
    repeat(front_echo_cycles) @(posedge clk);
    echo_front_tb = 1'b0;
end

always @(posedge trig_left) begin
    repeat(5) @(posedge clk);
    echo_left_tb = 1'b1;
    repeat(left_echo_cycles) @(posedge clk);
    echo_left_tb = 1'b0;
end

always @(posedge trig_right) begin
    repeat(5) @(posedge clk);
    echo_right_tb = 1'b1;
    repeat(right_echo_cycles) @(posedge clk);
    echo_right_tb = 1'b0;
end

// ---- Helpers ----------------------------------------------------------------
task wait_cycles;
    input [31:0] n;
    reg   [31:0] i;
    begin
        for (i = 0; i < n; i = i + 1)
            @(posedge clk);
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
        if (LED[3:0] === expected)
            $display("[%0t ns] PASS: State %0d reached", $time, expected);
        else
            $display("[%0t ns] FAIL: Timeout - expected state %0d, stuck in %0d",
                     $time, expected, LED[3:0]);
    end
endtask

// ---- State monitor ----------------------------------------------------------
reg [3:0] prev_state;
initial   prev_state = 4'hF;

always @(posedge clk) begin
    if (LED[3:0] !== prev_state) begin
        prev_state = LED[3:0];
        case (LED[3:0])
            4'd0: $display("[%0t ns] STATE -> FORWARD",    $time);
            4'd1: $display("[%0t ns] STATE -> STOP",       $time);
            4'd2: $display("[%0t ns] STATE -> CHECK",      $time);
            4'd3: $display("[%0t ns] STATE -> TURN_RIGHT", $time);
            4'd4: $display("[%0t ns] STATE -> TURN_LEFT",  $time);
            4'd5: $display("[%0t ns] STATE -> TURN_180",   $time);
            4'd6: $display("[%0t ns] STATE -> WAIT_180",   $time);
            4'd7: $display("[%0t ns] STATE -> RESUME",     $time);
            default: $display("[%0t ns] STATE -> UNKNOWN (%0d)", $time, LED[3:0]);
        endcase
    end
end

// ---- Main test sequence -----------------------------------------------------
initial begin
    $display("=== Object Avoiding Car Testbench START ===");

    KEY               = 2'b11;
    echo_front_tb     = 1'b0;
    echo_left_tb      = 1'b0;
    echo_right_tb     = 1'b0;
    front_echo_cycles = ECHO_500MM;
    left_echo_cycles  = ECHO_500MM;
    right_echo_cycles = ECHO_500MM;
    enc_la = 0; enc_lb = 0;
    enc_ra = 0; enc_rb = 0;

    // Reset
    KEY[0] = 1'b0;
    wait_cycles(32'd100);
    KEY[0] = 1'b1;
    wait_cycles(32'd100);
    $display("[%0t ns] Reset released", $time);

    // Sensor warmup
    $display("[%0t ns] Waiting for sensor warmup (1.5M cycles)...", $time);
    wait_cycles(32'd2_000_000);
    $display("[%0t ns] Sensors valid - starting tests", $time);

    // ------------------------------------------------------------------------
    // TEST 1: All clear -> stay FORWARD
    // ------------------------------------------------------------------------
    $display("\n--- TEST 1: All clear -> must stay in FORWARD ---");
    front_echo_cycles = ECHO_500MM;
    left_echo_cycles  = ECHO_500MM;
    right_echo_cycles = ECHO_500MM;
    wait_cycles(32'd1_300_000);
    $display("[%0t ns] Obstacle LEDs: front=%b left=%b right=%b (expect 0 0 0)",
             $time, LED[4], LED[5], LED[6]);
    if (LED[3:0] === 4'd0)
        $display("[%0t ns] PASS: Stayed in FORWARD", $time);
    else
        $display("[%0t ns] FAIL: Left FORWARD unexpectedly, state=%0d",
                 $time, LED[3:0]);

    // ------------------------------------------------------------------------
    // TEST 2: Front wall (200mm), sides clear -> TURN_RIGHT
    // ------------------------------------------------------------------------
    $display("\n--- TEST 2: Front wall 200mm, sides clear -> expect TURN_RIGHT ---");
    front_echo_cycles = ECHO_200MM;
    left_echo_cycles  = ECHO_500MM;
    right_echo_cycles = ECHO_500MM;

    wait_for_state(4'd1, 32'd1_500_000);  // STOP
    wait_for_state(4'd2, 32'd600_000);    // CHECK
    wait_for_state(4'd3, 32'd10_000);     // TURN_RIGHT
    wait_for_state(4'd7, 32'd1_600_000);  // RESUME
    wait_for_state(4'd0, 32'd600_000);    // FORWARD

    // clear again for next test
    front_echo_cycles = ECHO_500MM;
    wait_cycles(32'd700_000);

    // ------------------------------------------------------------------------
    // TEST 3: Front + right blocked, left clear -> TURN_LEFT
    // ------------------------------------------------------------------------
    $display("\n--- TEST 3: Front+Right blocked, left clear -> expect TURN_LEFT ---");
    front_echo_cycles = ECHO_150MM;
    right_echo_cycles = ECHO_150MM;
    left_echo_cycles  = ECHO_500MM;

    wait_for_state(4'd1, 32'd1_500_000);  // STOP
    wait_for_state(4'd2, 32'd600_000);    // CHECK
    wait_for_state(4'd4, 32'd10_000);     // TURN_LEFT
    wait_for_state(4'd7, 32'd1_600_000);  // RESUME
    wait_for_state(4'd0, 32'd600_000);    // FORWARD

    front_echo_cycles = ECHO_500MM;
    right_echo_cycles = ECHO_500MM;
    wait_cycles(32'd700_000);

    // ------------------------------------------------------------------------
    // TEST 4: All walls -> WAIT_180 then TURN_180
    // ------------------------------------------------------------------------
    $display("\n--- TEST 4: All walls blocked -> expect WAIT_180 -> TURN_180 ---");
    front_echo_cycles = ECHO_100MM;
    left_echo_cycles  = ECHO_100MM;
    right_echo_cycles = ECHO_100MM;

    wait_for_state(4'd1, 32'd1_500_000);  // STOP
    wait_for_state(4'd2, 32'd600_000);    // CHECK
    wait_for_state(4'd6, 32'd10_000);     // WAIT_180
    wait_for_state(4'd5, 32'd600_000);    // TURN_180
    wait_for_state(4'd7, 32'd3_200_000);  // RESUME
    wait_for_state(4'd0, 32'd600_000);    // FORWARD

    // ------------------------------------------------------------------------
    // TEST 5: All clear after everything -> FORWARD again
    //   Important: set ALL sensors to clear, wait until FSM returns to
    //   FORWARD, then do the motor pin check in that state.
    // ------------------------------------------------------------------------
    $display("\n--- TEST 5: All clear -> confirm back in FORWARD ---");
    front_echo_cycles = ECHO_500MM;
    left_echo_cycles  = ECHO_500MM;
    right_echo_cycles = ECHO_500MM;

    // Wait up to ~2 sensor periods for it to re-enter FORWARD
    wait_for_state(4'd0, 32'd1_300_000);

    wait_cycles(32'd500_000);  // small extra idle time
    if (LED[3:0] === 4'd0)
        $display("[%0t ns] PASS: Car in FORWARD - full scenario complete", $time);
    else
        $display("[%0t ns] FAIL: Expected FORWARD, got state %0d", $time, LED[3:0]);

    // Motor pins only make sense to check in FORWARD
    $display("\n--- MOTOR OUTPUT CHECK (expect FORWARD: IN1=1 IN2=0 IN3=1 IN4=0) ---");
    $display("motor_left_in1  (expect 1) = %b", motor_left_in1);
    $display("motor_left_in2  (expect 0) = %b", motor_left_in2);
    $display("motor_right_in3 (expect 1) = %b", motor_right_in3);
    $display("motor_right_in4 (expect 0) = %b", motor_right_in4);
    $display("motor_left_ena  (PWM)      = %b", motor_left_ena);
    $display("motor_right_enb (PWM)      = %b", motor_right_enb);

    if (motor_left_in1  && !motor_left_in2 &&
        motor_right_in3 && !motor_right_in4)
        $display("[%0t ns] PASS: Motor direction pins correct for FORWARD", $time);
    else
        $display("[%0t ns] FAIL: Motor direction pins incorrect", $time);

    $display("\n=== Testbench COMPLETE ===");
    $stop;
end

endmodule
