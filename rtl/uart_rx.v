// =============================================================================
// UART RECEIVER — 8N1
//
// Continuously listens on rx_in.  When a valid byte is received,
// rx_data is latched and rx_valid pulses HIGH for exactly one clock cycle.
//
// Sampling strategy: count to the MIDDLE of each bit (half-bit offset)
// to maximise setup/hold margin.
//
// Default: 50 MHz / 9600 = 5208 clocks per bit
// =============================================================================
module uart_rx #(
    parameter CLK_FREQ  = 32'd50_000_000,
    parameter BAUD_RATE = 32'd9600
)(
    input  wire       clk,
    input  wire       reset,       // Active LOW
    input  wire       rx_in,       // Serial input (idles HIGH)
    output reg  [7:0] rx_data,     // Received byte (valid when rx_valid=1)
    output reg        rx_valid     // One-cycle pulse on successful receive
);

localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

// State encoding
localparam [2:0]
    ST_IDLE     = 3'd0,
    ST_START    = 3'd1,
    ST_DATA     = 3'd2,
    ST_STOP     = 3'd3;

reg [2:0]  state;
reg [15:0] clk_cnt;
reg [2:0]  bit_idx;
reg [7:0]  rx_shift;

// 2-FF synchroniser for metastability protection
reg rx_s1, rx_s2;

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        rx_s1 <= 1'b1;
        rx_s2 <= 1'b1;
    end else begin
        rx_s1 <= rx_in;
        rx_s2 <= rx_s1;
    end
end

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        state    <= ST_IDLE;
        clk_cnt  <= 16'd0;
        bit_idx  <= 3'd0;
        rx_shift <= 8'd0;
        rx_data  <= 8'd0;
        rx_valid <= 1'b0;
    end else begin
        rx_valid <= 1'b0;   // Default: pulse off

        case (state)

            // ---------------------------------------------------------
            // IDLE: Wait for start bit (falling edge: HIGH → LOW)
            // ---------------------------------------------------------
            ST_IDLE: begin
                if (rx_s2 == 1'b0) begin
                    clk_cnt <= 16'd0;
                    state   <= ST_START;
                end
            end

            // ---------------------------------------------------------
            // START: Sample at mid-bit to confirm it's still LOW
            // ---------------------------------------------------------
            ST_START: begin
                if (clk_cnt == (CLKS_PER_BIT / 2) - 1) begin
                    if (rx_s2 == 1'b0) begin
                        // Valid start bit — reset counter for first data bit
                        clk_cnt <= 16'd0;
                        bit_idx <= 3'd0;
                        state   <= ST_DATA;
                    end else begin
                        // Glitch — go back to idle
                        state <= ST_IDLE;
                    end
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // ---------------------------------------------------------
            // DATA: Sample each bit at the middle of its period
            // ---------------------------------------------------------
            ST_DATA: begin
                if (clk_cnt == CLKS_PER_BIT - 1) begin
                    clk_cnt <= 16'd0;
                    rx_shift[bit_idx] <= rx_s2;   // LSB first
                    if (bit_idx == 3'd7)
                        state <= ST_STOP;
                    else
                        bit_idx <= bit_idx + 3'd1;
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // ---------------------------------------------------------
            // STOP: Wait for stop bit, then latch result
            // ---------------------------------------------------------
            ST_STOP: begin
                if (clk_cnt == CLKS_PER_BIT - 1) begin
                    if (rx_s2 == 1'b1) begin
                        // Valid stop bit — output the received byte
                        rx_data  <= rx_shift;
                        rx_valid <= 1'b1;
                    end
                    // Either way, return to idle
                    state   <= ST_IDLE;
                    clk_cnt <= 16'd0;
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
