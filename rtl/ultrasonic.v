module t1b_ultrasonic(
    input clk_50M, reset, echo_rx,
    output reg trig,
    output op,
    output wire [15:0] distance_out
);
initial begin
    trig = 0;
end
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////
    reg [19:0] next_trig_cnt;
    reg [9:0]  trig_cnt;
    reg [31:0] echo_cnt;
    reg        prev_echo;
    reg        measuring;
    reg [15:0] distance_reg;

    localparam integer START_DELAY_CYCLES = 51;
    localparam integer TRIG_HIGH_CYCLES   = 500;
    localparam integer TRIG_PERIOD_CYCLES = 3_250_000; //65ms

    assign distance_out = distance_reg;
    assign op = (distance_reg < 16'd70);

    initial begin
        next_trig_cnt = START_DELAY_CYCLES;
        trig_cnt      = 10'd0;
        echo_cnt      = 32'd0;
        prev_echo     = 1'b0;
        measuring     = 1'b0;
        distance_reg  = 16'hFFFF;   // FIX: was 16'dFFFF (invalid decimal)
    end

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            trig          <= 1'b0;
            next_trig_cnt <= START_DELAY_CYCLES;
            trig_cnt      <= 10'd0;
            echo_cnt      <= 32'd0;
            prev_echo     <= 1'b0;
            measuring     <= 1'b0;
            distance_reg  <= 16'hFFFF;   // FIX: was 16'dFFFF (invalid decimal)
        end else begin
            prev_echo <= echo_rx;

            if (trig_cnt > 10'd0) begin
                trig_cnt <= trig_cnt - 10'd1;
                trig     <= 1'b1;
            end else begin
                if (next_trig_cnt == 20'd0) begin
                    trig_cnt      <= TRIG_HIGH_CYCLES - 10'd1;
                    trig          <= 1'b1;
                    next_trig_cnt <= TRIG_PERIOD_CYCLES - TRIG_HIGH_CYCLES;
                end else begin
                    trig          <= 1'b0;
                    next_trig_cnt <= next_trig_cnt - 20'd1;
                end
            end

            if (!measuring) begin
                if (!prev_echo && echo_rx) begin
                    measuring <= 1'b1;
                    echo_cnt  <= 32'd1;
                end
            end else begin
                if (echo_rx) begin
                    echo_cnt <= echo_cnt + 32'd1;
                end else begin
                    measuring    <= 1'b0;
                    if ((echo_cnt * 32'd339) / 32'd100000 > 32'd65535)
                        distance_reg <= 16'hFFFF;
                    else
                        distance_reg <= ((echo_cnt * 32'd339) / 32'd100000);
                    echo_cnt <= 32'd0;
                end
            end
        end
    end
//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////
endmodule
