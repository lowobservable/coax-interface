`default_nettype none

`include "assert.v"
`include "mock_tx.v"

module coax_buffered_rx_tb;
    reg clk = 0;

    initial
    begin
        forever
        begin
            #1 clk <= ~clk;
        end
    end

    wire rx;

    mock_tx mock_tx (
        .tx(rx)
    );

    reg reset = 0;
    reg read_strobe = 0;

    coax_buffered_rx #(
        .CLOCKS_PER_BIT(8),
        .DEPTH(8)
    ) dut (
        .clk(clk),
        .reset(reset),
        .rx(rx),
        .read_strobe(read_strobe),
        .parity(1)
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

        mock_tx.tx_start_sequence;

        mock_tx.tx_word(10'b0000000001, 0);
        mock_tx.tx_word(10'b0000000010, 0);
        mock_tx.tx_word(10'b0000000011, 1);
        mock_tx.tx_word(10'b0000000100, 0);
        mock_tx.tx_word(10'b0000000101, 1);
        mock_tx.tx_word(10'b0000000110, 1);
        mock_tx.tx_word(10'b0000000111, 0);
        mock_tx.tx_word(10'b0000001000, 0);

        mock_tx.tx_end_sequence;

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

        mock_tx.tx_start_sequence;

        mock_tx.tx_word(10'b0000000001, 0);
        mock_tx.tx_word(10'b0000000010, 0);
        mock_tx.tx_word(10'b0000000011, 1);
        mock_tx.tx_word(10'b0000000100, 0);

        mock_tx.tx_end_sequence;

        #8;

        `assert_low(dut.error, "error should be LOW");

        mock_tx.tx_start_sequence;
        mock_tx.tx_word(10'b0000000101, 1);
        mock_tx.tx_end_sequence;

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

        mock_tx.tx_start_sequence;

        repeat (9)
        begin
            mock_tx.tx_word(10'b0000000000, 1);
        end

        mock_tx.tx_end_sequence;

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
endmodule
