`default_nettype none

`include "assert.v"

module control_tb;
    reg clk = 0;

    initial
    begin
        forever
        begin
            #1 clk <= ~clk;
        end
    end

    reg reset = 0;
    reg spi_cs = 1;
    reg [7:0] spi_rx_data;
    reg spi_rx_strobe = 0;
    reg rx_active = 0;
    reg rx_error = 0;
    reg [9:0] rx_data = 0;
    reg rx_empty = 1;
    reg rx_full = 0;

    control dut (
        .clk(clk),
        .reset(reset),
        .spi_cs(spi_cs),
        .spi_rx_data(spi_rx_data),
        .spi_rx_strobe(spi_rx_strobe),
        .rx_active(rx_active),
        .rx_error(rx_error),
        .rx_data(rx_data),
        .rx_empty(rx_empty),
        .rx_full(rx_full)
    );

    initial
    begin
        $dumpfile("control_tb.vcd");
        $dumpvars(0, control_tb);

        test_1;

        $finish;
    end

    task test_1;
    begin
        $display("START: test_1");

        dut_reset;

        #2;

        spi_cs = 0;
        spi_rx_data = 8'h05; // RX
        spi_rx_strobe = 1;
        #2;
        spi_rx_strobe = 0;

        #16;

        spi_rx_strobe = 1;
        #2;
        spi_rx_strobe = 0;

        #16;

        repeat (8)
        begin
            spi_rx_strobe = 1;
            #2;
            spi_rx_strobe = 0;

            #16;

            spi_rx_strobe = 1;
            #2;
            spi_rx_strobe = 0;

            #16;
        end

        spi_cs = 1;

        #64;

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
