`default_nettype none

`include "assert.v"

module coax_buffered_tx_tb;
    reg clk = 0;

    initial
    begin
        forever
        begin
            #1 clk <= ~clk;
        end
    end

    reg reset = 0;
    reg [9:0] data;
    reg load_strobe = 0;
    reg start_strobe = 0;

    coax_buffered_tx #(
        .CLOCKS_PER_BIT(8),
        .DEPTH(8)
    ) dut (
        .clk(clk),
        .reset(reset),
        .data(data),
        .load_strobe(load_strobe),
        .start_strobe(start_strobe)
    );

    initial
    begin
        $dumpfile("coax_buffered_tx_tb.vcd");
        $dumpvars(0, coax_buffered_tx_tb);

        test_1;

        $finish;
    end

    task test_1;
    begin
        $display("START: test_1");

        dut_reset;

        #8;

        data = 10'b0101110101;
        load_strobe = 1;
        #2;
        load_strobe = 0;

        #8;

        data = 10'b1010001110;
        load_strobe = 1;
        #2;
        load_strobe = 0;

        #8;

        data = 10'b0101110101;
        load_strobe = 1;
        #2;
        load_strobe = 0;

        #8;

        start_strobe = 1;
        #2;
        start_strobe = 0;

        #1000;

        $display("END: test_1");
    end
    endtask

    task dut_reset;
    begin
        reset = 1;
        #2;
        reset = 0;
    end
    endtask
endmodule
