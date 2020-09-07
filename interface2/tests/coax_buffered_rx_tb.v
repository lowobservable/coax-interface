`default_nettype none

`include "assert.v"

module coax_buffered_rx_tb;
    reg clk = 0;

    initial
    begin
        forever
        begin
            #1 clk <= ~clk;
        end
    end

    reg rx = 0;
    reg reset = 0;
    reg read_strobe = 0;

    coax_buffered_rx #(
        .CLOCKS_PER_BIT(8),
        .DEPTH(8)
    ) dut (
        .clk(clk),
        .reset(reset),
        .rx(rx),
        .read_strobe(read_strobe)
    );

    initial
    begin
        $dumpfile("coax_buffered_rx_tb.vcd");
        $dumpvars(0, coax_buffered_rx_tb);

        test_1;
        test_2;
        test_3;

        $finish;
    end

    task test_1;
    begin
        $display("START: test_1");

        dut_reset;

        #2;

        rx_start_sequence;

        rx_word(10'b0000000001, 0);
        rx_word(10'b0000000010, 0);
        rx_word(10'b0000000011, 1);
        rx_word(10'b0000000100, 0);
        rx_word(10'b0000000101, 1);
        rx_word(10'b0000000110, 1);
        rx_word(10'b0000000111, 0);
        rx_word(10'b0000001000, 0);

        rx_end_sequence;

        #8;

        `assert_high(dut.full, "full should be HIGH");
        `assert_low(dut.empty, "empty should be LOW");

        `assert_low(dut.error, "error should be LOW");

        repeat (8)
        begin
            read_strobe = 1;
            #2;
            read_strobe = 0;

            #8;
        end

        `assert_low(dut.full, "full should be LOW");
        `assert_high(dut.empty, "empty should be HIGH");

        #64;

        $display("END: test_1");
    end
    endtask

    task test_2;
    begin
        $display("START: test_2");

        dut_reset;

        #2;

        rx_start_sequence;

        rx_word(10'b0000000001, 0);
        rx_word(10'b0000000010, 0);
        rx_word(10'b0000000011, 1);
        rx_word(10'b0000000100, 0);

        rx_end_sequence;

        #8;

        `assert_low(dut.error, "error should be LOW");

        rx_start_sequence;
        rx_word(10'b0000000101, 1);
        rx_end_sequence;

        #8;

        `assert_high(dut.error, "error should be HIGH");
        `assert_equal(dut.data, dut.ERROR_OVERFLOW, "error should be ERROR_OVERFLOW");

        #64;

        $display("END: test_2");
    end
    endtask

    task test_3;
    begin
        $display("START: test_3");

        dut_reset;

        #2;

        rx_start_sequence;

        repeat (9)
        begin
            rx_word(10'b0000000000, 1);
        end

        rx_end_sequence;

        #8;

        `assert_high(dut.error, "error should be HIGH");
        `assert_equal(dut.data, dut.ERROR_OVERFLOW, "error should be ERROR_OVERFLOW");

        `assert_high(dut.full, "full should be HIGH");
        `assert_low(dut.empty, "empty should be LOW");

        #64;

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

    task rx_bit (
        input bit
    );
    begin
        rx_bit_custom(bit, 8, 8);
    end
    endtask

    task rx_start_sequence;
    begin
        rx = 0;
        #16;
        rx = 1;
        #16;
        rx = 0;

        rx_bit(1);
        rx_bit(1);
        rx_bit(1);
        rx_bit(1);
        rx_bit(1);

        rx = 0;
        #24;
        rx = 1;
        #24;
    end
    endtask

    task rx_word (
        input [9:0] data,
        input parity
    );
    begin
        rx_bit(1);

        rx_bit(data[9]);
        rx_bit(data[8]);
        rx_bit(data[7]);
        rx_bit(data[6]);
        rx_bit(data[5]);
        rx_bit(data[4]);
        rx_bit(data[3]);
        rx_bit(data[2]);
        rx_bit(data[1]);
        rx_bit(data[0]);

        rx_bit(parity);
    end
    endtask

    task rx_end_sequence;
    begin
        rx_bit(0);

        rx = 1;
        #16;
        rx = 0;
    end
    endtask

    task rx_bit_custom (
        input bit,
        input [15:0] first_half_duration,
        input [15:0] second_half_duration
    );
    begin
        rx = !bit;
        #first_half_duration;
        rx = bit;
        #second_half_duration;
        rx = 0;
    end
    endtask
endmodule
