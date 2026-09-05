// =============================================================================
// MOTOR CONTROLLER - Object Avoidance State Machine
// Hardware: TB6612FNG + DC 300RPM Geared Motors + HW-870 IR Sensor
//
// States:
//   S_FORWARD    (0): Both motors forward
//   S_STOP       (1): All motors off, wait STOP_WAIT_CYCLES
//   S_CHECK      (2): Latch side sensors, decide turn direction
//   S_TURN_RIGHT (3): Left fwd + Right bwd, run TURN_90_CYCLES
//   S_TURN_LEFT  (4): Right fwd + Left bwd, run TURN_90_CYCLES
//   S_TURN_180   (5): Spin right, run TURN_180_CYCLES
//   S_WAIT_180   (6): All stop, wait STOP_WAIT_CYCLES before 180
//   S_RESUME     (7): Both motors forward, run STOP_WAIT_CYCLES then FORWARD
//   S_IDLE       (8): Motors off, waiting for START command (initial state)
//   S_MAZE_DONE  (9): Maze completed, motors off, maze_done flag raised
//
// Commands (from ESP32 via UART):
//   cmd_start  : S_IDLE      -> S_FORWARD
//   cmd_stop   : any state   -> S_IDLE
//   cmd_return : S_MAZE_DONE -> S_TURN_180 (then normal avoidance resumes)
//
// IR sensor:
//   ir_maze_end == 1 (active HIGH) in S_FORWARD -> S_MAZE_DONE
// =============================================================================
module motor_controller #(
    parameter FRONT_THRESHOLD  = 16'd250,       // mm - front wall detect
    parameter SIDE_THRESHOLD   = 16'd200,       // mm - side wall detect

    // -----------------------------------------------------------------------
    // Simulation-length timing defaults (override from top-level for HW)
    // -----------------------------------------------------------------------
    parameter TURN_90_CYCLES   = 32'd750_000,   // 15ms  @ 50MHz
    parameter TURN_180_CYCLES  = 32'd1_500_000, // 30ms  @ 50MHz
    parameter STOP_WAIT_CYCLES = 32'd250_000,   //  5ms  @ 50MHz
    parameter SENSOR_WARMUP    = 32'd1_500_000, // 30ms  @ 50MHz

    // -----------------------------------------------------------------------
    // Per-motor speed trim
    // PWM counter is 6-bit (period=64), step = 1.5625%
    // -----------------------------------------------------------------------
    parameter SPEED_FULL_L     = 6'd12,
    parameter SPEED_FULL_R     = 6'd10,
    parameter SPEED_TURN_L     = 6'd12,
    parameter SPEED_TURN_R     = 6'd10,

    // IR sensor polarity: 1 = active LOW (LOW = black detected)
    //                     0 = active HIGH (HIGH = black detected)
    parameter IR_ACTIVE_LOW       = 0,
    parameter IR_DEBOUNCE_CYCLES  = 32'd50      // 1us @ 50MHz
)(
    input  wire        clk,
    input  wire        reset,           // Active LOW

    // Ultrasonic distances
    input  wire [15:0] dist_front,
    input  wire [15:0] dist_left,
    input  wire [15:0] dist_right,

    // IR sensor for maze end detection
    input  wire        ir_maze_end,     // From HW-870 DO pin

    // Commands from ESP32 (active-HIGH, one-cycle pulses)
    input  wire        cmd_start,
    input  wire        cmd_stop,
    input  wire        cmd_return,

    // Motor duty outputs
    output reg  [5:0]  duty_left,
    output reg  [5:0]  duty_right,

    // Motor direction outputs
    output reg         dir_left_fwd,
    output reg         dir_left_bwd,
    output reg         dir_right_fwd,
    output reg         dir_right_bwd,

    // Status outputs
    output reg  [3:0]  state_out,
    output reg         maze_done        // HIGH when in S_MAZE_DONE
);

// =============================================================================
// STATE ENCODING
// =============================================================================
localparam [3:0]
    S_FORWARD    = 4'd0,
    S_STOP       = 4'd1,
    S_CHECK      = 4'd2,
    S_TURN_RIGHT = 4'd3,
    S_TURN_LEFT  = 4'd4,
    S_TURN_180   = 4'd5,
    S_WAIT_180   = 4'd6,
    S_RESUME     = 4'd7,
    S_IDLE       = 4'd8,
    S_MAZE_DONE  = 4'd9;

reg [3:0]  state;
reg [31:0] timer;

// Warmup guard
reg [31:0] warmup_timer;
reg        sensors_valid;

// Side wall latches
reg wall_left_latch;
reg wall_right_latch;

// Live threshold comparisons
wire wall_front = (dist_front < FRONT_THRESHOLD);
wire wall_left  = (dist_left  < SIDE_THRESHOLD);
wire wall_right = (dist_right < SIDE_THRESHOLD);

// IR sensor synchronizer & debounce logic (glitch rejection)
reg        ir_s1, ir_s2;
reg [31:0] ir_debounce_cnt;
reg        ir_debounced;

wire ir_raw_active = IR_ACTIVE_LOW ? (~ir_s2) : ir_s2;

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        ir_s1           <= 1'b0;
        ir_s2           <= 1'b0;
        ir_debounce_cnt <= 32'd0;
        ir_debounced    <= 1'b0;
    end else begin
        ir_s1 <= ir_maze_end;
        ir_s2 <= ir_s1;

        if (ir_raw_active) begin
            if (ir_debounce_cnt >= IR_DEBOUNCE_CYCLES)
                ir_debounced <= 1'b1;
            else
                ir_debounce_cnt <= ir_debounce_cnt + 32'd1;
        end else begin
            ir_debounce_cnt <= 32'd0;
            ir_debounced    <= 1'b0;
        end
    end
end

// =============================================================================
// STATE REGISTER
// =============================================================================
always @(posedge clk or negedge reset) begin
    if (!reset) begin
        state            <= S_IDLE;     // Start in IDLE — wait for START cmd
        timer            <= 32'd0;
        warmup_timer     <= 32'd0;
        sensors_valid    <= 1'b0;
        wall_left_latch  <= 1'b0;
        wall_right_latch <= 1'b0;
        maze_done        <= 1'b0;
    end else begin

        // ------------------------------------------------------------------
        // Warmup counter - blocks wall detection until sensors settle
        // ------------------------------------------------------------------
        if (!sensors_valid) begin
            if (warmup_timer >= SENSOR_WARMUP)
                sensors_valid <= 1'b1;
            else
                warmup_timer <= warmup_timer + 32'd1;
        end

        // ------------------------------------------------------------------
        // GLOBAL COMMAND: cmd_stop from ANY state -> S_IDLE
        // ------------------------------------------------------------------
        if (cmd_stop) begin
            state     <= S_IDLE;
            timer     <= 32'd0;
            maze_done <= 1'b0;
        end
        // ------------------------------------------------------------------
        // GLOBAL IR DETECTION: Black detected in ANY active running state -> S_MAZE_DONE
        // ------------------------------------------------------------------
        else if (ir_debounced && (state != S_IDLE) && (state != S_MAZE_DONE)) begin
            state     <= S_MAZE_DONE;
            timer     <= 32'd0;
            maze_done <= 1'b1;
        end else begin

            case (state)

                // ------------------------------------------------------------
                // IDLE: Motors off, wait for START command
                // ------------------------------------------------------------
                S_IDLE: begin
                    timer     <= 32'd0;
                    maze_done <= 1'b0;
                    if (cmd_start)
                        state <= S_FORWARD;
                end

                // ------------------------------------------------------------
                // FORWARD: Drive until front wall OR IR maze-end marker
                // ------------------------------------------------------------
                S_FORWARD: begin
                    timer <= 32'd0;
                    if (wall_front && sensors_valid) begin
                        state <= S_STOP;
                    end
                end

                // ------------------------------------------------------------
                // STOP: Hold motors off, count STOP_WAIT_CYCLES
                // ------------------------------------------------------------
                S_STOP: begin
                    if (timer >= STOP_WAIT_CYCLES) begin
                        wall_left_latch  <= wall_left;
                        wall_right_latch <= wall_right;
                        timer <= 32'd0;
                        state <= S_CHECK;
                    end else begin
                        timer <= timer + 32'd1;
                    end
                end

                // ------------------------------------------------------------
                // CHECK: Decide direction from latched side sensor readings
                // ------------------------------------------------------------
                S_CHECK: begin
                    timer <= 32'd0;
                    if (!wall_right_latch)
                        state <= S_TURN_RIGHT;
                    else if (!wall_left_latch)
                        state <= S_TURN_LEFT;
                    else
                        state <= S_WAIT_180;
                end

                // ------------------------------------------------------------
                // TURN_RIGHT: Clockwise spin for TURN_90_CYCLES
                // ------------------------------------------------------------
                S_TURN_RIGHT: begin
                    if (timer >= TURN_90_CYCLES) begin
                        timer <= 32'd0;
                        state <= S_RESUME;
                    end else
                        timer <= timer + 32'd1;
                end

                // ------------------------------------------------------------
                // TURN_LEFT: Counter-clockwise spin for TURN_90_CYCLES
                // ------------------------------------------------------------
                S_TURN_LEFT: begin
                    if (timer >= TURN_90_CYCLES) begin
                        timer <= 32'd0;
                        state <= S_RESUME;
                    end else
                        timer <= timer + 32'd1;
                end

                // ------------------------------------------------------------
                // WAIT_180: Extra stop pause before spinning 180
                // ------------------------------------------------------------
                S_WAIT_180: begin
                    if (timer >= STOP_WAIT_CYCLES) begin
                        timer <= 32'd0;
                        state <= S_TURN_180;
                    end else
                        timer <= timer + 32'd1;
                end

                // ------------------------------------------------------------
                // TURN_180: Clockwise spin for TURN_180_CYCLES
                // ------------------------------------------------------------
                S_TURN_180: begin
                    if (timer >= TURN_180_CYCLES) begin
                        timer <= 32'd0;
                        state <= S_RESUME;
                    end else
                        timer <= timer + 32'd1;
                end

                // ------------------------------------------------------------
                // RESUME: Short forward burst to clear turn geometry
                // ------------------------------------------------------------
                S_RESUME: begin
                    if (timer >= STOP_WAIT_CYCLES) begin
                        timer <= 32'd0;
                        state <= S_FORWARD;
                    end else
                        timer <= timer + 32'd1;
                end

                // ------------------------------------------------------------
                // MAZE_DONE: Motors off, wait for RETURN command
                // ------------------------------------------------------------
                S_MAZE_DONE: begin
                    timer     <= 32'd0;
                    maze_done <= 1'b1;
                    if (cmd_return) begin
                        maze_done <= 1'b0;
                        state     <= S_WAIT_180;  // Turn 180, then resume avoidance
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end
end

// =============================================================================
// OUTPUT LOGIC
// =============================================================================
always @(*) begin
    // Defaults: motors off
    duty_left     = 6'd0;
    duty_right    = 6'd0;
    dir_left_fwd  = 1'b0;
    dir_left_bwd  = 1'b0;
    dir_right_fwd = 1'b0;
    dir_right_bwd = 1'b0;
    state_out     = state;

    case (state)

        S_FORWARD, S_RESUME: begin
            duty_left     = SPEED_FULL_L;
            duty_right    = SPEED_FULL_R;
            dir_left_fwd  = 1'b1;
            dir_left_bwd  = 1'b0;
            dir_right_fwd = 1'b1;
            dir_right_bwd = 1'b0;
        end

        S_IDLE, S_MAZE_DONE, S_STOP, S_CHECK, S_WAIT_180: begin
            duty_left     = 6'd0;
            duty_right    = 6'd0;
            dir_left_fwd  = 1'b0;
            dir_left_bwd  = 1'b0;
            dir_right_fwd = 1'b0;
            dir_right_bwd = 1'b0;
        end

        S_TURN_RIGHT: begin
            duty_left     = SPEED_TURN_L;
            duty_right    = SPEED_TURN_R;
            dir_left_fwd  = 1'b1;   // Left forward
            dir_left_bwd  = 1'b0;
            dir_right_fwd = 1'b0;
            dir_right_bwd = 1'b1;   // Right backward -> clockwise
        end

        S_TURN_LEFT: begin
            duty_left     = SPEED_TURN_L;
            duty_right    = SPEED_TURN_R;
            dir_left_fwd  = 1'b0;
            dir_left_bwd  = 1'b1;   // Left backward
            dir_right_fwd = 1'b1;   // Right forward -> counter-clockwise
            dir_right_bwd = 1'b0;
        end

        S_TURN_180: begin
            duty_left     = SPEED_TURN_L;
            duty_right    = SPEED_TURN_R;
            dir_left_fwd  = 1'b1;   // Same spin as TURN_RIGHT
            dir_left_bwd  = 1'b0;
            dir_right_fwd = 1'b0;
            dir_right_bwd = 1'b1;
        end

        default: begin
            duty_left     = 6'd0;
            duty_right    = 6'd0;
            dir_left_fwd  = 1'b0;
            dir_left_bwd  = 1'b0;
            dir_right_fwd = 1'b0;
            dir_right_bwd = 1'b0;
        end
    endcase
end

endmodule
