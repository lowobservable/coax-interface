// Copyright (c) 2020, Andrew Kay
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted, provided that the above
// copyright notice and this permission notice appear in all copies.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
// WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
// MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
// ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
// WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
// ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
// OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

`default_nettype none

module control (
    input clk,
    input reset,

    // SPI
    input spi_cs,
    input [7:0] spi_rx_data,
    input spi_rx_strobe,
    output reg [7:0] spi_tx_data,
    output reg spi_tx_strobe,

    output loopback,

    // TX
    output reg tx_reset,
    input tx_active,
    output reg [9:0] tx_data,
    output reg tx_load_strobe,
    output reg tx_start_strobe,
    input tx_empty,
    input tx_full,
    input tx_ready,
    output tx_parity,
    
    // RX
    output reg rx_reset,
    input rx_active,
    input rx_error,
    input [9:0] rx_data,
    output reg rx_read_strobe,
    input rx_empty,
    output rx_parity
);
    localparam STATE_IDLE = 0;
    localparam STATE_READ_REGISTER_1 = 1;
    localparam STATE_READ_REGISTER_2 = 2;
    localparam STATE_WRITE_REGISTER_1 = 3;
    localparam STATE_WRITE_REGISTER_2 = 4;
    localparam STATE_TX_1 = 5;
    localparam STATE_TX_2 = 6;
    localparam STATE_TX_3 = 7;
    localparam STATE_RX_1 = 8;
    localparam STATE_RX_2 = 9;
    localparam STATE_RX_3 = 10;
    localparam STATE_RX_4 = 11;
    localparam STATE_RESET = 12;

    reg [7:0] state = STATE_IDLE;
    reg [7:0] next_state;

    reg [7:0] control_register = 8'b01001000;
    reg [7:0] next_control_register;
    reg [7:0] register_mask;
    reg [7:0] next_register_mask;

    reg [7:0] command;
    reg [7:0] next_command;

    reg [7:0] next_spi_tx_data;
    reg next_spi_tx_strobe;

    reg next_tx_reset;
    reg previous_tx_active;
    reg [9:0] next_tx_data;
    reg tx_data_valid = 0;
    reg next_tx_data_valid;
    reg next_tx_load_strobe;
    reg next_tx_start_strobe;
    reg tx_complete = 0;
    reg next_tx_complete;

    reg next_rx_reset;
    reg next_rx_read_strobe;
    reg [15:0] rx_buffer;
    reg [15:0] next_rx_buffer;

    always @(*)
    begin
        next_state = state;

        next_control_register = control_register;
        next_register_mask = register_mask;

        next_command = command;

        next_spi_tx_data = spi_tx_data;
        next_spi_tx_strobe = 0;

        next_tx_reset = 0;
        next_tx_data = tx_data;
        next_tx_data_valid = tx_data_valid;
        next_tx_load_strobe = 0;
        next_tx_start_strobe = 0;
        next_tx_complete = tx_complete;

        next_rx_reset = 0;
        next_rx_read_strobe = 0;
        next_rx_buffer = rx_buffer;

        case (state)
            STATE_IDLE:
            begin
                if (spi_rx_strobe)
                begin
                    next_command = spi_rx_data;

                    case (spi_rx_data[3:0])
                        4'h2: next_state = STATE_READ_REGISTER_1;
                        4'h3: next_state = STATE_WRITE_REGISTER_1;
                        4'h4: next_state = STATE_TX_1;
                        4'h5: next_state = STATE_RX_1;
                        4'hf: next_state = STATE_RESET;
                    endcase
                end
            end

            STATE_READ_REGISTER_1:
            begin
                next_spi_tx_data = 0;

                case (command[7:4])
                    4'h1: next_spi_tx_data = { 1'b0, rx_error, rx_active, 1'b0, tx_complete, tx_active, 2'b0 };
                    4'h2: next_spi_tx_data = control_register;
                    4'hf: next_spi_tx_data = 8'ha5;
                endcase

                next_spi_tx_strobe = 1;

                next_state = STATE_READ_REGISTER_2;
            end

            STATE_READ_REGISTER_2:
            begin
                if (spi_rx_strobe)
                    next_state = STATE_READ_REGISTER_1;
            end

            STATE_WRITE_REGISTER_1:
            begin
                if (spi_rx_strobe)
                begin
                    next_register_mask = spi_rx_data;

                    next_state = STATE_WRITE_REGISTER_2;
                end
            end

            STATE_WRITE_REGISTER_2:
            begin
                if (spi_rx_strobe)
                begin
                    case (command[7:4])
                        4'h2: next_control_register = (control_register & ~register_mask) | (spi_rx_data & register_mask);
                    endcase

                    next_state = STATE_IDLE;
                end
            end

            STATE_TX_1:
            begin
                next_tx_complete = 0;
                next_state = STATE_TX_2;
            end

            STATE_TX_2:
            begin
                if (spi_rx_strobe)
                begin
                    next_tx_data_valid = 0;

                    if (tx_full)
                    begin
                        next_spi_tx_data = 8'b10000001; // Overflow
                        next_spi_tx_strobe = 1;
                    end
                    else if (!tx_ready)
                    begin
                        next_spi_tx_data = 8'b10000010; // Underflow
                        next_spi_tx_strobe = 1;
                    end
                    else
                    begin
                        next_tx_data = { spi_rx_data[1:0], 8'b00000000 };
                        next_tx_data_valid = 1;

                        next_spi_tx_data = 8'h00;
                        next_spi_tx_strobe = 1;
                    end

                    next_state = STATE_TX_3;
                end
            end

            STATE_TX_3:
            begin
                if (spi_rx_strobe)
                begin
                    next_tx_data = { tx_data[9:8], spi_rx_data };
                    next_tx_load_strobe = tx_data_valid;

                    next_state = STATE_TX_2;
                end
            end

            STATE_RX_1:
            begin
                next_rx_buffer = { rx_error, rx_empty, 4'b0000, rx_data };

                next_state = STATE_RX_2;
            end

            STATE_RX_2:
            begin
                next_spi_tx_data = rx_buffer[15:8];
                next_spi_tx_strobe = 1;

                next_state = STATE_RX_3;
            end

            STATE_RX_3:
            begin
                if (spi_rx_strobe)
                begin
                    next_spi_tx_data = rx_buffer[7:0];
                    next_spi_tx_strobe = 1;

                    // Reset on error and only dequeue if not empty.
                    if (rx_buffer[15]) 
                        next_rx_reset = 1; // TODO: should this be more explicit?
                    else if (!rx_buffer[14])
                        next_rx_read_strobe = 1;

                    next_state = STATE_RX_4;
                end
            end

            STATE_RX_4:
            begin
                if (spi_rx_strobe)
                    next_state = STATE_RX_1;
            end

            STATE_RESET:
            begin
                // TODO: should this also reset the control register flags
                // like loopback?

                next_tx_reset = 1;
                next_tx_complete = 0;

                next_rx_reset = 1;

                next_state = STATE_IDLE;
            end
        endcase

        if (spi_cs)
        begin
            if (!tx_empty && !tx_active)
                next_tx_start_strobe = 1; // TODO: this can last for multiple clock cycles, but that is okay...

            next_state = STATE_IDLE;
        end

        if (!tx_active && previous_tx_active)
            next_tx_complete = 1;
    end

    always @(posedge clk)
    begin
        state <= next_state;

        control_register <= next_control_register;
        register_mask <= next_register_mask;

        command <= next_command;

        spi_tx_data <= next_spi_tx_data;
        spi_tx_strobe <= next_spi_tx_strobe;

        tx_reset <= next_tx_reset;
        tx_data <= next_tx_data;
        tx_data_valid <= next_tx_data_valid;
        tx_load_strobe <= next_tx_load_strobe;
        tx_start_strobe <= next_tx_start_strobe;
        tx_complete <= next_tx_complete;

        rx_reset <= next_rx_reset;
        rx_read_strobe <= next_rx_read_strobe;
        rx_buffer <= next_rx_buffer;

        if (reset)
        begin
            state <= STATE_IDLE;

            control_register <= 8'b01001000;

            command <= 0;

            spi_tx_data <= 0;
            spi_tx_strobe <= 0;

            tx_reset <= 0;
            tx_data <= 0;
            tx_load_strobe <= 0;
            tx_start_strobe <= 0;
            tx_complete <= 0;

            rx_reset <= 0;
            rx_read_strobe <= 0;
            rx_buffer <= 0;
        end

        previous_tx_active <= tx_active;
    end

    assign loopback = control_register[0];

    assign tx_parity = control_register[3];
    assign rx_parity = control_register[6];
endmodule
