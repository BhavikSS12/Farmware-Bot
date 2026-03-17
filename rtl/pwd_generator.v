// =============================================================================
// PWM GENERATOR MODULE
// FIX: All blocking (=) assignments inside clocked always replaced with
//      non-blocking (<=) to eliminate simulation race conditions
// =============================================================================
module pwm_generator(
    input  wire        clk_3125KHz,
    input  wire [3:0]  duty_cycle,
    output reg         clk_195KHz,
    output reg         pwm_signal
);

initial begin
    clk_195KHz = 1'b0;
    pwm_signal = 1'b1;
end
//////////////////DO NOT MAKE ANY CHANGES ABOVE THIS LINE //////////////////

reg [3:0] pwm_counter = 4'd0;
reg [2:0] clk_counter = 3'd0;

always @(posedge clk_3125KHz) begin
    // FIX: non-blocking assignments throughout
    clk_counter <= clk_counter + 3'd1;
    if (clk_counter == 3'd0)
        clk_195KHz <= ~clk_195KHz;   // FIX: toggle after increment (was before)

    // PWM: HIGH when counter < duty, LOW otherwise
    if (pwm_counter < duty_cycle)
        pwm_signal <= 1'b1;
    else
        pwm_signal <= 1'b0;

    pwm_counter <= pwm_counter + 4'd1;  // wraps 15->0 automatically
end

//////////////////DO NOT MAKE ANY CHANGES BELOW THIS LINE //////////////////
endmodule
