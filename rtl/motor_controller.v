// =============================================================================
// MOTOR CONTROLLER - Object Avoidance State Machine
//
// FIXES:
//   FIX 1: Simulation-safe default parameters hardcoded (no +define+SIM needed)
//          Change these back to large values for real hardware deployment
//   FIX 2: sensors_valid warmup guard prevents false wall at reset (dist=0)
//   FIX 3: Removed unused wall_front_latch (dead logic)
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
// =============================================================================
module motor_controller #(
    parameter FRONT_THRESHOLD  = 16'd250,       // mm - front wall detect
    parameter SIDE_THRESHOLD   = 16'd200,       // mm - side wall detect

    // -----------------------------------------------------------------------
    // FIX 1: Simulation-length timing (uncomment hardware values for real HW)
    // -----------------------------------------------------------------------
    // -- SIMULATION (ModelSim) --
    parameter TURN_90_CYCLES   = 32'd750_000,   // 15ms  @ 50MHz
    parameter TURN_180_CYCLES  = 32'd1_500_000, // 30ms  @ 50MHz
    parameter STOP_WAIT_CYCLES = 32'd250_000,   //  5ms  @ 50MHz
    parameter SENSOR_WARMUP    = 32'd1_500_000, // 30ms  @ 50MHz

    // -- HARDWARE (uncomment and comment above for real deployment) --
    // parameter TURN_90_CYCLES   = 32'd75_000_000,
    // parameter TURN_180_CYCLES  = 32'd150_000_000,
    // parameter STOP_WAIT_CYCLES = 32'd25_000_000,
    // parameter SENSOR_WARMUP    = 32'd1_500_000,

    parameter SPEED_FULL       = 4'd12,
    parameter SPEED_TURN       = 4'd10
)(
    input  wire        clk,
    input  wire        reset,           // Active LOW

    input  wire [15:0] dist_front,
    input  wire [15:0] dist_left,
    input  wire [15:0] dist_right,

    output reg  [3:0]  duty_left,
    output reg  [3:0]  duty_right,

    output reg         dir_left_fwd,
    output reg         dir_left_bwd,
    output reg         dir_right_fwd,
    output reg         dir_right_bwd,

    output reg  [3:0]  state_out
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
    S_RESUME     = 4'd7;

reg [3:0]  state;
reg [31:0] timer;

// FIX 2: Warmup guard
reg [31:0] warmup_timer;
reg        sensors_valid;

// FIX 3: Only side wall latches (front latch was never read in S_CHECK)
reg wall_left_latch;
reg wall_right_latch;

// Live threshold comparisons
wire wall_front = (dist_front < FRONT_THRESHOLD);
wire wall_left  = (dist_left  < SIDE_THRESHOLD);
wire wall_right = (dist_right < SIDE_THRESHOLD);

// =============================================================================
// STATE REGISTER
// =============================================================================
always @(posedge clk or negedge reset) begin
    if (!reset) begin
        state            <= S_FORWARD;
        timer            <= 32'd0;
        warmup_timer     <= 32'd0;
        sensors_valid    <= 1'b0;
        wall_left_latch  <= 1'b0;
        wall_right_latch <= 1'b0;
    end else begin

        // ------------------------------------------------------------------
        // FIX 2: Warmup counter - blocks wall detection until sensors settle
        // dist_reg initialised to 0xFFFF so no false trip, but guard adds
        // extra safety for any edge case at power-on
        // ------------------------------------------------------------------
        if (!sensors_valid) begin
            if (warmup_timer >= SENSOR_WARMUP)
                sensors_valid <= 1'b1;
            else
                warmup_timer <= warmup_timer + 32'd1;
        end

        case (state)

            // ----------------------------------------------------------------
            // FORWARD: Drive until front wall seen AND sensors are valid
            // ----------------------------------------------------------------
            S_FORWARD: begin
                timer <= 32'd0;
                if (wall_front && sensors_valid)
                    state <= S_STOP;
            end

            // ----------------------------------------------------------------
            // STOP: Hold motors off, count STOP_WAIT_CYCLES, latch side walls
            // ----------------------------------------------------------------
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

            // ----------------------------------------------------------------
            // CHECK: Decide direction from latched side sensor readings
            // ----------------------------------------------------------------
            S_CHECK: begin
                timer <= 32'd0;
                if (!wall_right_latch)
                    state <= S_TURN_RIGHT;
                else if (!wall_left_latch)
                    state <= S_TURN_LEFT;
                else
                    state <= S_WAIT_180;
            end

            // ----------------------------------------------------------------
            // TURN_RIGHT: Clockwise spin for TURN_90_CYCLES
            // ----------------------------------------------------------------
            S_TURN_RIGHT: begin
                if (timer >= TURN_90_CYCLES) begin
                    timer <= 32'd0;
                    state <= S_RESUME;
                end else
                    timer <= timer + 32'd1;
            end

            // ----------------------------------------------------------------
            // TURN_LEFT: Counter-clockwise spin for TURN_90_CYCLES
            // ----------------------------------------------------------------
            S_TURN_LEFT: begin
                if (timer >= TURN_90_CYCLES) begin
                    timer <= 32'd0;
                    state <= S_RESUME;
                end else
                    timer <= timer + 32'd1;
            end

            // ----------------------------------------------------------------
            // WAIT_180: Extra stop pause before spinning 180
            // ----------------------------------------------------------------
            S_WAIT_180: begin
                if (timer >= STOP_WAIT_CYCLES) begin
                    timer <= 32'd0;
                    state <= S_TURN_180;
                end else
                    timer <= timer + 32'd1;
            end

            // ----------------------------------------------------------------
            // TURN_180: Clockwise spin for TURN_180_CYCLES (~2x TURN_90)
            // ----------------------------------------------------------------
            S_TURN_180: begin
                if (timer >= TURN_180_CYCLES) begin
                    timer <= 32'd0;
                    state <= S_RESUME;
                end else
                    timer <= timer + 32'd1;
            end

            // ----------------------------------------------------------------
            // RESUME: Short forward burst to clear turn geometry
            // ----------------------------------------------------------------
            S_RESUME: begin
                if (timer >= STOP_WAIT_CYCLES) begin
                    timer <= 32'd0;
                    state <= S_FORWARD;
                end else
                    timer <= timer + 32'd1;
            end

            default: state <= S_FORWARD;
        endcase
    end
end

// =============================================================================
// OUTPUT LOGIC
// =============================================================================
always @(*) begin
    duty_left     = 4'd0;
    duty_right    = 4'd0;
    dir_left_fwd  = 1'b0;
    dir_left_bwd  = 1'b0;
    dir_right_fwd = 1'b0;
    dir_right_bwd = 1'b0;
    state_out     = state;

    case (state)

        S_FORWARD, S_RESUME: begin
            duty_left     = SPEED_FULL;
            duty_right    = SPEED_FULL;
            dir_left_fwd  = 1'b1;
            dir_left_bwd  = 1'b0;
            dir_right_fwd = 1'b1;
            dir_right_bwd = 1'b0;
        end

        S_STOP, S_CHECK, S_WAIT_180: begin
            duty_left     = 4'd0;
            duty_right    = 4'd0;
            dir_left_fwd  = 1'b0;
            dir_left_bwd  = 1'b0;
            dir_right_fwd = 1'b0;
            dir_right_bwd = 1'b0;
        end

        S_TURN_RIGHT: begin
            duty_left     = SPEED_TURN;
            duty_right    = SPEED_TURN;
            dir_left_fwd  = 1'b1;   // Left forward
            dir_left_bwd  = 1'b0;
            dir_right_fwd = 1'b0;
            dir_right_bwd = 1'b1;   // Right backward -> clockwise
        end

        S_TURN_LEFT: begin
            duty_left     = SPEED_TURN;
            duty_right    = SPEED_TURN;
            dir_left_fwd  = 1'b0;
            dir_left_bwd  = 1'b1;   // Left backward
            dir_right_fwd = 1'b1;   // Right forward -> counter-clockwise
            dir_right_bwd = 1'b0;
        end

        S_TURN_180: begin
            duty_left     = SPEED_TURN;
            duty_right    = SPEED_TURN;
            dir_left_fwd  = 1'b1;   // Same spin as TURN_RIGHT
            dir_left_bwd  = 1'b0;
            dir_right_fwd = 1'b0;
            dir_right_bwd = 1'b1;
        end

        default: begin
            duty_left     = 4'd0;
            duty_right    = 4'd0;
            dir_left_fwd  = 1'b0;
            dir_left_bwd  = 1'b0;
            dir_right_fwd = 1'b0;
            dir_right_bwd = 1'b0;
        end
    endcase
end

endmodule
