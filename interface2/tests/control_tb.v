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

    control dut (
        .clk(clk),
        .reset(reset),
        .spi_cs(spi_cs),
        .spi_rx_data(spi_rx_data),
        .spi_rx_strobe(spi_rx_strobe),
        .rx_active(rx_active),
        .rx_error(rx_error),
        .rx_data(rx_data),
        .rx_empty(rx_empty)
    );

    initial
    begin
        $dumpfile("control_tb.vcd");
        $dumpvars(0, control_tb);

        //test_1;
        test_2;

        $finish;
    end

    task test_1;
    begin
        $display("START: test_1");

        dut_reset;

        #2;

        rx_data = 10'b1111111111;
        rx_empty = 0;

        spi_cs = 0;

        spi_send(8'h05); // RX

        #16;

        spi_send(8'h00);

        #16;

        rx_data = 10'b0000000000;
        rx_empty = 1;

        spi_send(8'h00);

        #16;

        repeat (8)
        begin
            spi_send(8'h00);

            #16;

            spi_send(8'h00);

            #16;
        end

        spi_cs = 1;

        #64;

        $display("END: test_1");
    end
    endtask

    task test_2;
    begin
        $display("START: test_2");

        dut_reset;

        #2;

        spi_cs = 0;

        spi_send(8'h04); // TX

        #16;

        spi_send(8'b00000011);

        #16;

        spi_send(8'b11111111);

        #16;

        #16;

        spi_cs = 1;

        #64;

        $display("END: test_2");
    end
    endtask

    task dut_reset;
    begin
        reset = 1;
        #2;
        reset = 0;
    end
    endtask

    task spi_send (
        input [7:0] data
    );
    begin
        spi_rx_data = data;
        spi_rx_strobe = 1;
        #2;
        spi_rx_strobe = 0;
    end
    endtask
endmodule
