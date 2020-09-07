`default_nettype none

`include "mock_tx.v"

module coax_rx_bit_timer_tb;
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
    wire sample;
    wire synchronized;

    coax_rx_bit_timer #(
        .CLOCKS_PER_BIT(8)
    ) dut (
        .clk(clk),
        .rx(rx),
        .reset(reset),
        .sample(sample),
        .synchronized(synchronized)
    );

    initial
    begin
        $dumpfile("coax_rx_bit_timer_tb.vcd");
        $dumpvars(0, coax_rx_bit_timer_tb);

        // Idle
        #32;

        // Perfect
        mock_tx.tx_bit(1);
        mock_tx.tx_bit(0);
        mock_tx.tx_bit(1);

        // Delayed
        mock_tx.tx_bit_custom(0, 9, 8);

        // Shortened
        mock_tx.tx_bit_custom(0, 6, 7);

        // Stuck
        mock_tx.tx_bit_custom(1, 24, 8);

        // Reset
        dut_reset;

        mock_tx.tx_bit(1);

        #32;

        $finish;
    end

    task dut_reset;
    begin
        reset = 1;
        #2;
        reset = 0;
    end
    endtask
endmodule
