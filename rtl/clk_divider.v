// =============================================================================
// CLOCK DIVIDER
// =============================================================================
module clk_divider #(
    parameter DIVIDE_BY = 312
)(
    input  wire clk_in,
    input  wire reset,
    output reg  clk_out
);

reg [$clog2(DIVIDE_BY)-1:0] counter;

initial begin
    clk_out = 1'b0;
    counter  = 0;
end

always @(posedge clk_in or negedge reset) begin
    if (!reset) begin
        clk_out <= 1'b0;
        counter  <= 0;
    end else begin
        if (counter == (DIVIDE_BY/2) - 1) begin
            clk_out <= ~clk_out;
            counter  <= 0;
        end else begin
            counter <= counter + 1;
        end
    end
end

endmodule
