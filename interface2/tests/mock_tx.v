module mock_tx (
    output reg tx = 0
);
    task tx_bit (
        input bit
    );
    begin
        tx_bit_custom(bit, 8, 8);
    end
    endtask

    task tx_start_sequence;
    begin
        tx = 0;
        #16;
        tx = 1;
        #16;
        tx = 0;

        tx_bit(1);
        tx_bit(1);
        tx_bit(1);
        tx_bit(1);
        tx_bit(1);

        tx = 0;
        #24;
        tx = 1;
        #24;
    end
    endtask

    task tx_word (
        input [9:0] data,
        input parity
    );
    begin
        tx_bit(1);

        tx_bit(data[9]);
        tx_bit(data[8]);
        tx_bit(data[7]);
        tx_bit(data[6]);
        tx_bit(data[5]);
        tx_bit(data[4]);
        tx_bit(data[3]);
        tx_bit(data[2]);
        tx_bit(data[1]);
        tx_bit(data[0]);

        tx_bit(parity);
    end
    endtask

    task tx_end_sequence;
    begin
        tx_bit(0);

        tx = 1;
        #16;
        tx = 0;
    end
    endtask

    task tx_bit_custom (
        input bit,
        input [15:0] first_half_duration,
        input [15:0] second_half_duration
    );
    begin
        tx = !bit;
        #first_half_duration;
        tx = bit;
        #second_half_duration;
        tx = 0;
    end
    endtask

    task tx_set (
        input value
    );
    begin
        tx = value;
    end
    endtask
endmodule
