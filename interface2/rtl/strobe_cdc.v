`default_nettype none

module strobe_cdc (
    input clk_in,
    input strobe_in,
    input clk_out,
    output reg strobe_out
);
    reg toggle_in;

    always @(posedge clk_in)
    begin
        if (strobe_in)
            toggle_in <= ~toggle_in;
    end

    reg [2:0] toggle_out;

    always @(posedge clk_out)
    begin
        toggle_out <= { toggle_out[1:0], toggle_in };

        strobe_out <= 0;

        if (toggle_out[2] != toggle_out[1])
            strobe_out <= 1;
    end
endmodule
