// =============================================================================
// UART TRANSMITTER — 8N1
//
// Sends one byte when tx_start is pulsed HIGH.
// tx_busy stays HIGH while transmission is in progress.
// tx_out idles HIGH (standard UART idle).
//
// Baud rate = CLK_FREQ / BAUD_RATE  clocks per bit
// Default: 50 MHz / 9600 = 5208 clocks per bit
// =============================================================================
module uart_tx #(
    parameter CLK_FREQ  = 32'd50_000_000,
    parameter BAUD_RATE = 32'd9600
)(
    input  wire       clk,
    input  wire       reset,       // Active LOW
    input  wire [7:0] tx_data,     // Byte to send
    input  wire       tx_start,    // Pulse HIGH to begin
    output reg        tx_out,      // Serial output (idles HIGH)
    output wire       tx_busy      // HIGH while transmitting
);

localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

// State encoding
localparam [2:0]
    ST_IDLE     = 3'd0,
    ST_START    = 3'd1,
    ST_DATA     = 3'd2,
    ST_STOP     = 3'd3;

reg [2:0]  state;
reg [15:0] clk_cnt;    // Counts clocks within one bit period
reg [2:0]  bit_idx;    // Which data bit (0..7)
reg [7:0]  tx_shift;   // Latched copy of tx_data

assign tx_busy = (state != ST_IDLE);

always @(posedge clk or negedge reset) begin
    if (!reset) begin
        state    <= ST_IDLE;
        tx_out   <= 1'b1;       // UART idle = HIGH
        clk_cnt  <= 16'd0;
        bit_idx  <= 3'd0;
        tx_shift <= 8'd0;
    end else begin
        case (state)

            // ---------------------------------------------------------
            // IDLE: Wait for tx_start pulse
            // ---------------------------------------------------------
            ST_IDLE: begin
                tx_out <= 1'b1;
                if (tx_start) begin
                    tx_shift <= tx_data;   // Latch the byte
                    clk_cnt  <= 16'd0;
                    state    <= ST_START;
                end
            end

            // ---------------------------------------------------------
            // START BIT: Drive LOW for one bit period
            // ---------------------------------------------------------
            ST_START: begin
                tx_out <= 1'b0;
                if (clk_cnt == CLKS_PER_BIT - 1) begin
                    clk_cnt <= 16'd0;
                    bit_idx <= 3'd0;
                    state   <= ST_DATA;
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // ---------------------------------------------------------
            // DATA BITS: Send LSB first (8 bits)
            // ---------------------------------------------------------
            ST_DATA: begin
                tx_out <= tx_shift[bit_idx];
                if (clk_cnt == CLKS_PER_BIT - 1) begin
                    clk_cnt <= 16'd0;
                    if (bit_idx == 3'd7)
                        state <= ST_STOP;
                    else
                        bit_idx <= bit_idx + 3'd1;
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            // ---------------------------------------------------------
            // STOP BIT: Drive HIGH for one bit period
            // ---------------------------------------------------------
            ST_STOP: begin
                tx_out <= 1'b1;
                if (clk_cnt == CLKS_PER_BIT - 1) begin
                    clk_cnt <= 16'd0;
                    state   <= ST_IDLE;
                end else begin
                    clk_cnt <= clk_cnt + 16'd1;
                end
            end

            default: state <= ST_IDLE;
        endcase
    end
end

endmodule
