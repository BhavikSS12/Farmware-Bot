// =============================================================================
// QUADRATURE ENCODER MODULE for N20 Motor
//
// Reads a standard quadrature encoder (channel A + B) and outputs:
//   - 32-bit signed tick count (up/down depending on direction)
//   - direction flag (1 = forward, 0 = backward)
//   - velocity estimate (ticks per sample window)
//
// Quadrature decoding truth table:
//   prev_A prev_B | new_A new_B | Count
//     0      0    |   0    1    |  +1
//     0      1    |   1    1    |  +1
//     1      1    |   1    0    |  +1
//     1      0    |   0    0    |  +1
//     (and reverse for -1)
//
// Usage: Instantiate one per motor. Useful for:
//   - Measuring actual distance traveled
//   - PID speed control
//   - Verifying turns are complete
// =============================================================================
module encoder(
    input  wire        clk,
    input  wire        reset,      // Active LOW
    input  wire        enc_a,      // Encoder channel A
    input  wire        enc_b,      // Encoder channel B
    output reg  [31:0] tick_count, // Signed tick count (two's complement)
    output reg         direction,  // 1=forward (count up), 0=backward
    output reg  [15:0] velocity    // Ticks in last sample window
);

// Debounce / synchronize encoder inputs (2 FF synchronizer)
reg enc_a_s1, enc_a_s2;
reg enc_b_s1, enc_b_s2;

// Previous state for edge detection
reg prev_a, prev_b;

// Velocity measurement
reg [31:0] tick_prev;
reg [31:0] vel_timer;
localparam VEL_WINDOW = 32'd5_000_000; // Measure velocity every 100ms at 50MHz

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        enc_a_s1   <= 1'b0;
        enc_a_s2   <= 1'b0;
        enc_b_s1   <= 1'b0;
        enc_b_s2   <= 1'b0;
        prev_a     <= 1'b0;
        prev_b     <= 1'b0;
        tick_count <= 32'd0;
        direction  <= 1'b1;
        velocity   <= 16'd0;
        tick_prev  <= 32'd0;
        vel_timer  <= 32'd0;
    end else begin
        // 2-stage synchronizer to avoid metastability
        enc_a_s1 <= enc_a;
        enc_a_s2 <= enc_a_s1;
        enc_b_s1 <= enc_b;
        enc_b_s2 <= enc_b_s1;

        prev_a <= enc_a_s2;
        prev_b <= enc_b_s2;

        // Quadrature decoding - detect transitions on A
        if (enc_a_s2 != prev_a) begin
            if (enc_a_s2 == 1'b1) begin
                // Rising edge on A
                if (enc_b_s2 == 1'b0) begin
                    tick_count <= tick_count + 32'd1;
                    direction  <= 1'b1;
                end else begin
                    tick_count <= tick_count - 32'd1;
                    direction  <= 1'b0;
                end
            end else begin
                // Falling edge on A
                if (enc_b_s2 == 1'b1) begin
                    tick_count <= tick_count + 32'd1;
                    direction  <= 1'b1;
                end else begin
                    tick_count <= tick_count - 32'd1;
                    direction  <= 1'b0;
                end
            end
        end

        // Velocity measurement
        if (vel_timer >= VEL_WINDOW) begin
            vel_timer <= 32'd0;
            // Difference in ticks = velocity
            if (tick_count >= tick_prev)
                velocity <= (tick_count - tick_prev > 32'd65535) ? 16'hFFFF
                           : tick_count[15:0] - tick_prev[15:0];
            else
                velocity <= (tick_prev - tick_count > 32'd65535) ? 16'hFFFF
                           : tick_prev[15:0] - tick_count[15:0];
            tick_prev <= tick_count;
        end else begin
            vel_timer <= vel_timer + 32'd1;
        end
    end
end

endmodule