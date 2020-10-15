`default_nettype none

`include "assert.v"

module coax_tx_tb;
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
    reg strobe = 0;

    coax_tx #(
        .CLOCKS_PER_BIT(8)
    ) dut (
        .clk(clk),
        .reset(reset),
        .data(data),
        .strobe(strobe),
        .parity(1)
    );

    initial
    begin
        $dumpfile("coax_tx_tb.vcd");
        $dumpvars(0, coax_tx_tb);

        test_1;
        test_2;
        test_3;

        $finish;
    end

    task test_1;
    begin
        $display("START: test_1");

        `assert_equal(dut.state, dut.IDLE, "state should be IDLE");

        dut_reset;

        #8;

        `assert_equal(dut.state, dut.IDLE, "state should be IDLE");

        $display("END: test_1");
    end
    endtask

    task test_2;
    begin
        $display("START: test_2");

        `assert_equal(dut.state, dut.IDLE, "state should be IDLE");

        data = 10'b0101110101;

        strobe = 1;
        #2;
        strobe = 0;

        #400;

        `assert_equal(dut.state, dut.IDLE, "state should be IDLE");

        $display("END: test_2");
    end
    endtask

    task test_3;
    begin
        $display("START: test_3");

        `assert_equal(dut.state, dut.IDLE, "state should be IDLE");

        data = 10'b0101110101;

        strobe = 1;
        #2;
        strobe = 0;

        #330;

        data = 10'b1010001110;

        strobe = 1;
        #2;
        strobe = 0;

        #600;

        `assert_equal(dut.state, dut.IDLE, "state should be IDLE");

        $display("END: test_3");
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
