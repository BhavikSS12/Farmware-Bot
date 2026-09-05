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
    reg [23:0] next_trig_cnt;
    reg [9:0]  trig_cnt;
    reg [31:0] echo_cnt;
    reg        echo_s1, echo_s2;
    reg        prev_echo;
    reg        measuring;
    reg [15:0] distance_reg;

    localparam integer START_DELAY_CYCLES = 51;
    localparam integer TRIG_HIGH_CYCLES   = 500;
    localparam integer TRIG_PERIOD_CYCLES = 3_250_000; // 65ms @ 50MHz
    localparam integer ECHO_TIMEOUT       = 2_000_000; // 40ms timeout (~6.8m)
    localparam integer MIN_VALID_ECHO     = 2_950;     // ~10mm minimum (rejects noise spikes)

    assign distance_out = distance_reg;
    assign op = (distance_reg < 16'd70);

    initial begin
        next_trig_cnt = START_DELAY_CYCLES;
        trig_cnt      = 10'd0;
        echo_cnt      = 32'd0;
        echo_s1       = 1'b0;
        echo_s2       = 1'b0;
        prev_echo     = 1'b0;
        measuring     = 1'b0;
        distance_reg  = 16'hFFFF;
    end

    always @(posedge clk_50M or negedge reset) begin
        if (!reset) begin
            trig          <= 1'b0;
            next_trig_cnt <= START_DELAY_CYCLES;
            trig_cnt      <= 10'd0;
            echo_cnt      <= 32'd0;
            echo_s1       <= 1'b0;
            echo_s2       <= 1'b0;
            prev_echo     <= 1'b0;
            measuring     <= 1'b0;
            distance_reg  <= 16'hFFFF;
        end else begin
            // 2-FF synchronizer for external echo input
            echo_s1   <= echo_rx;
            echo_s2   <= echo_s1;
            prev_echo <= echo_s2;

            // Trigger pulse generator (10us pulse every 65ms)
            if (trig_cnt > 10'd0) begin
                trig_cnt <= trig_cnt - 10'd1;
                trig     <= 1'b1;
            end else begin
                if (next_trig_cnt == 24'd0) begin
                    trig_cnt      <= TRIG_HIGH_CYCLES - 10'd1;
                    trig          <= 1'b1;
                    next_trig_cnt <= TRIG_PERIOD_CYCLES - TRIG_HIGH_CYCLES;
                end else begin
                    trig          <= 1'b0;
                    next_trig_cnt <= next_trig_cnt - 24'd1;
                end
            end

            // Echo pulse width measurement
            if (!measuring) begin
                if (!prev_echo && echo_s2) begin
                    measuring <= 1'b1;
                    echo_cnt  <= 32'd1;
                end
            end else begin
                if (echo_s2) begin
                    if (echo_cnt >= ECHO_TIMEOUT) begin
                        // Timeout: no object in range / echo lost
                        measuring    <= 1'b0;
                        distance_reg <= 16'hFFFF;
                        echo_cnt     <= 32'd0;
                    end else begin
                        echo_cnt <= echo_cnt + 32'd1;
                    end
                end else begin
                    // Echo falling edge: calculate distance in mm
                    measuring <= 1'b0;
                    if (echo_cnt < MIN_VALID_ECHO) begin
                        // Glitch/noise (<1cm) -> ignore as out of range
                        distance_reg <= 16'hFFFF;
                    end else if ((echo_cnt * 32'd339) / 32'd100000 > 32'd65535) begin
                        distance_reg <= 16'hFFFF;
                    end else begin
                        distance_reg <= ((echo_cnt * 32'd339) / 32'd100000);
                    end
                    echo_cnt <= 32'd0;
                end
            end
        end
    end
//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////
endmodule
