`default_nettype none

`include "assert.v"

module coax_rx_tb;
    reg clk = 0;

    initial
    begin
        forever
        begin
            #1 clk <= ~clk;
        end
    end

    reg reset = 0;
    reg rx = 0;

    coax_rx #(
        .CLOCKS_PER_BIT(8)
    ) dut (
        .clk(clk),
        .reset(reset),
        .rx(rx)
    );

    initial
    begin
        $dumpfile("coax_rx_tb.vcd");
        $dumpvars(0, coax_rx_tb);

        test_1;
        test_2;
        test_3;
        test_4;
        test_5;
        test_6;
        test_7;
        test_8;
        test_9;
        test_10;
        test_11;
        test_12;
        test_13;
        test_14;
        test_15;
        test_16;

        $finish;
    end

    task test_1;
    begin
        $display("START: test_1");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        dut_reset;

        #8;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_1");
    end
    endtask

    task test_2;
    begin
        $display("START: test_2");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx = 1;

        #64;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_2");
    end
    endtask

    task test_3;
    begin
        $display("START: test_3");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx = 1;
        #16;
        rx = 0;

        rx_bit(1);

        #64;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_3");
    end
    endtask

    task test_4;
    begin
        $display("START: test_4");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx = 1;
        #16;
        rx = 0;

        rx_bit(1);
        rx_bit(1);

        #64;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_4");
    end
    endtask

    task test_5;
    begin
        $display("START: test_5");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx = 1;
        #16;
        rx = 0;

        rx_bit(1);
        rx_bit(1);
        rx_bit(1);

        #64;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_5");
    end
    endtask

    task test_6;
    begin
        $display("START: test_6");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx = 1;
        #16;
        rx = 0;

        rx_bit(1);
        rx_bit(1);
        rx_bit(1);
        rx_bit(1);

        #64;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_6");
    end
    endtask

    task test_7;
    begin
        $display("START: test_7");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx = 1;
        #16;
        rx = 0;

        rx_bit(1);
        rx_bit(1);
        rx_bit(1);
        rx_bit(1);
        rx_bit(1);

        #64;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_7");
    end
    endtask

    task test_8;
    begin
        $display("START: test_8");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

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

        #64;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx = 0;

        $display("END: test_8");
    end
    endtask

    task test_9;
    begin
        $display("START: test_9");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx_start_sequence;

        rx = 0;

        #64;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_9");
    end
    endtask

    task test_10;
    begin
        $display("START: test_10");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx_start_sequence;
        rx_bit(1); // SYNC_BIT

        #64;

        `assert_equal(dut.state, dut.STATE_ERROR, "State should be STATE_ERROR");

        `assert_high(dut.error, "error should be HIGH");
        `assert_equal(dut.data, dut.ERROR_LOSS_OF_MID_BIT_TRANSITION, "data should be ERROR_LOSS_OF_MID_BIT_TRANSITION");

        dut_reset;

        #16;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_10");
    end
    endtask

    task test_11;
    begin
        $display("START: test_11");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx_start_sequence;
        rx_bit(1); // SYNC_BIT
        rx_bit(0); // MSB DATA_BIT
        rx_bit(1);
        rx_bit(1);
        rx_bit(0);
        rx_bit(1);

        #64;

        `assert_equal(dut.state, dut.STATE_ERROR, "State should be STATE_ERROR");

        `assert_high(dut.error, "error should be HIGH");
        `assert_equal(dut.data, dut.ERROR_LOSS_OF_MID_BIT_TRANSITION, "data should be ERROR_LOSS_OF_MID_BIT_TRANSITION");

        dut_reset;

        #16;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_11");
    end
    endtask

    task test_12;
    begin
        $display("START: test_12");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx_start_sequence;
        rx_bit(1); // SYNC_BIT
        rx_bit(0); // MSB DATA_BIT
        rx_bit(1);
        rx_bit(1);
        rx_bit(0);
        rx_bit(1);
        rx_bit(1);
        rx_bit(0);
        rx_bit(0);
        rx_bit(1);
        rx_bit(1); // LSB DATA_BIT

        #64;

        `assert_equal(dut.state, dut.STATE_ERROR, "State should be STATE_ERROR");

        `assert_high(dut.error, "error should be HIGH");
        `assert_equal(dut.data, dut.ERROR_LOSS_OF_MID_BIT_TRANSITION, "data should be ERROR_LOSS_OF_MID_BIT_TRANSITION");

        dut_reset;

        #16;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_12");
    end
    endtask

    task test_13;
    begin
        $display("START: test_13");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx_start_sequence;
        rx_bit(1); // SYNC_BIT
        rx_bit(0); // MSB DATA_BIT
        rx_bit(1);
        rx_bit(1);
        rx_bit(0);
        rx_bit(1);
        rx_bit(1);
        rx_bit(0);
        rx_bit(0);
        rx_bit(1);
        rx_bit(1); // LSB DATA_BIT
        rx_bit(0); // PARITY_BIT

        #64;

        `assert_equal(dut.state, dut.STATE_ERROR, "State should be STATE_ERROR");

        `assert_high(dut.error, "error should be HIGH");
        `assert_equal(dut.data, dut.ERROR_PARITY, "data should be ERROR_PARITY");

        dut_reset;

        #16;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_13");
    end
    endtask

    task test_14;
    begin
        $display("START: test_14");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx_start_sequence;
        rx_bit(1); // SYNC_BIT
        rx_bit(0); // MSB DATA_BIT
        rx_bit(1);
        rx_bit(1);
        rx_bit(0);
        rx_bit(1);
        rx_bit(1);
        rx_bit(0);
        rx_bit(0);
        rx_bit(1);
        rx_bit(1); // LSB DATA_BIT
        rx_bit(1); // PARITY_BIT

        #64;

        `assert_equal(dut.state, dut.STATE_ERROR, "State should be STATE_ERROR");

        `assert_high(dut.error, "error should be HIGH");
        `assert_equal(dut.data, dut.ERROR_LOSS_OF_MID_BIT_TRANSITION, "data should be ERROR_LOSS_OF_MID_BIT_TRANSITION");

        dut_reset;

        #16;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_14");
    end
    endtask

    task test_15;
    begin
        $display("START: test_15");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx_start_sequence;
        rx_bit(1); // SYNC_BIT
        rx_bit(0); // MSB DATA_BIT
        rx_bit(1);
        rx_bit(1);
        rx_bit(0);
        rx_bit(1);
        rx_bit(1);
        rx_bit(0);
        rx_bit(0);
        rx_bit(1);
        rx_bit(1); // LSB DATA_BIT
        rx_bit(1); // PARITY_BIT
        rx_bit(0);

        #64;

        `assert_equal(dut.state, dut.STATE_ERROR, "State should be STATE_ERROR");

        `assert_high(dut.error, "error should be HIGH");
        `assert_equal(dut.data, dut.ERROR_INVALID_END_SEQUENCE, "data should be INVALID_END_SEQUENCE_STATE_ERROR");

        dut_reset;

        #16;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        $display("END: test_15");
    end
    endtask

    task test_16;
    begin
        $display("START: test_16");

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        rx_start_sequence;
        rx_bit(1); // SYNC_BIT
        rx_bit(0); // MSB DATA_BIT
        rx_bit(1);
        rx_bit(1);
        rx_bit(0);
        rx_bit(1);
        rx_bit(1);
        rx_bit(0);
        rx_bit(0);
        rx_bit(1);
        rx_bit(1); // LSB DATA_BIT
        rx_bit(1); // PARITY_BIT
        rx_end_sequence;

        #64;

        `assert_equal(dut.state, dut.STATE_IDLE, "state should be STATE_IDLE");

        `assert_equal(dut.data, 10'b0110110011, "data not correct")
        
        $display("END: test_16");
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
