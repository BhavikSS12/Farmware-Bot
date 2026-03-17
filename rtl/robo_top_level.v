// =============================================================================
// TOP LEVEL MODULE - Object Avoiding Car
// DE0-Nano FPGA + 3x HC-SR04 + 2x N20 Motor + 2x L298N Driver
//
// FIX: TURN_90_CYCLES, TURN_180_CYCLES, STOP_WAIT_CYCLES changed to
//      simulation-length values so motor_controller is not overridden
//      with 75M/150M/25M hardware values during ModelSim simulation.
//      Restore hardware values before synthesis for real deployment.
// =============================================================================
module robo_top_level(
    input  wire        CLOCK_50,
    input  wire [1:0]  KEY,
    output wire [7:0]  LED,

    output wire        trig_left,
    input  wire        echo_left,
    output wire        trig_front,
    input  wire        echo_front,
    output wire        trig_right,
    input  wire        echo_right,

    output wire        motor_left_in1,
    output wire        motor_left_in2,
    output wire        motor_left_ena,

    output wire        motor_right_in3,
    output wire        motor_right_in4,
    output wire        motor_right_enb
);

// =============================================================================
// PARAMETERS
// Detection thresholds - same for sim and hardware
// =============================================================================
localparam FRONT_THRESHOLD  = 16'd250;
localparam SIDE_THRESHOLD   = 16'd200;

// =============================================================================

// For real hardware ength timing parameters
localparam TURN_90_CYCLES   = 32'd75_000_000;    // HW: 1.5s
localparam TURN_180_CYCLES  = 32'd150_000_000;  // HW: 3.0s
localparam STOP_WAIT_CYCLES = 32'd25_000_000;    // HW: 0.5s
// =============================================================================

// Simulation-length timing parameters
//localparam TURN_90_CYCLES   = 32'd750_000;    // SIM: 15ms 
//localparam TURN_180_CYCLES  = 32'd1_500_000;  // SIM: 30ms
//localparam STOP_WAIT_CYCLES = 32'd250_000;    // SIM: 5ms 

localparam SPEED_FULL       = 4'd12;
localparam SPEED_TURN       = 4'd10;

// =============================================================================
// INTERNAL SIGNALS
// =============================================================================
wire reset = KEY[0];   // Active LOW

wire clk_3125KHz;
wire clk_195KHz_left, clk_195KHz_right;

wire [15:0] dist_left, dist_front, dist_right;
wire        obj_left,  obj_front,  obj_right;

wire [3:0]  duty_left,      duty_right;
wire        dir_left_fwd,   dir_left_bwd;
wire        dir_right_fwd,  dir_right_bwd;

wire pwm_left, pwm_right;

// =============================================================================
// CLOCK DIVIDER: 50MHz -> 3.125MHz (divide by 16)
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
// MOTOR CONTROLLER STATE MACHINE
// All timing parameters explicitly passed so localparams above take effect
// =============================================================================
motor_controller #(
    .FRONT_THRESHOLD (FRONT_THRESHOLD),
    .SIDE_THRESHOLD  (SIDE_THRESHOLD),
    .TURN_90_CYCLES  (TURN_90_CYCLES),    // 750_000  (sim)
    .TURN_180_CYCLES (TURN_180_CYCLES),   // 1_500_000 (sim)
    .STOP_WAIT_CYCLES(STOP_WAIT_CYCLES),  // 250_000  (sim)
    .SPEED_FULL      (SPEED_FULL),
    .SPEED_TURN      (SPEED_TURN)
) u_motor_ctrl(
    .clk             (CLOCK_50),
    .reset           (reset),
    .dist_front      (dist_front),
    .dist_left       (dist_left),
    .dist_right      (dist_right),
    .duty_left       (duty_left),
    .duty_right      (duty_right),
    .dir_left_fwd    (dir_left_fwd),
    .dir_left_bwd    (dir_left_bwd),
    .dir_right_fwd   (dir_right_fwd),
    .dir_right_bwd   (dir_right_bwd),
    .state_out       (LED[3:0])
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
// MOTOR OUTPUT ASSIGNMENTS
// =============================================================================
assign motor_left_in1  = dir_left_fwd;
assign motor_left_in2  = dir_left_bwd;
assign motor_left_ena  = pwm_left;

assign motor_right_in3 = dir_right_fwd;
assign motor_right_in4 = dir_right_bwd;
assign motor_right_enb = pwm_right;

// =============================================================================
// DEBUG LEDs
//   [3:0] = state machine state
//   [4]   = front obstacle
//   [5]   = left obstacle
//   [6]   = right obstacle
//   [7]   = reset active
// =============================================================================
assign LED[4] = (dist_front < FRONT_THRESHOLD);
assign LED[5] = (dist_left  < SIDE_THRESHOLD);
assign LED[6] = (dist_right < SIDE_THRESHOLD);
assign LED[7] = ~reset;

endmodule
