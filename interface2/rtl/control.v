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

    // TX
    
    // RX
    output reg rx_reset,
    input rx_active,
    input rx_error,
    input [9:0] rx_data,
    output reg rx_read_strobe,
    input rx_empty
);
    localparam STATE_IDLE = 0;
    localparam STATE_READ_REGISTER_1 = 1;
    localparam STATE_READ_REGISTER_2 = 2;
    localparam STATE_RX_1 = 3;
    localparam STATE_RX_2 = 4;
    localparam STATE_RX_3 = 5;
    localparam STATE_RX_4 = 6;

    reg [7:0] state = STATE_IDLE;
    reg [7:0] next_state;

    reg [7:0] command;
    reg [7:0] next_command;

    reg [7:0] next_spi_tx_data;
    reg next_spi_tx_strobe;

    reg next_rx_reset;
    reg next_rx_read_strobe;
    reg [15:0] rx_buffer;
    reg [15:0] next_rx_buffer;

    always @(*)
    begin
        next_state = state;

        next_command = command;

        next_spi_tx_data = spi_tx_data;
        next_spi_tx_strobe = 0;

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
                        4'h5: next_state = STATE_RX_1;
                    endcase
                end
            end

            STATE_READ_REGISTER_1:
            begin
                next_spi_tx_data = 0;

                case (command[7:4])
                    4'h1: next_spi_tx_data = { 1'b0, rx_error, rx_active, 5'b0 };
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
        endcase

        if (spi_cs)
            next_state = STATE_IDLE;
    end

    always @(posedge clk)
    begin
        state <= next_state;

        command <= next_command;

        spi_tx_data <= next_spi_tx_data;
        spi_tx_strobe <= next_spi_tx_strobe;

        rx_reset <= next_rx_reset;
        rx_read_strobe <= next_rx_read_strobe;
        rx_buffer <= next_rx_buffer;

        if (reset)
        begin
            state <= STATE_IDLE;

            command <= 0;

            spi_tx_data <= 0;
            spi_tx_strobe <= 0;

            rx_reset <= 0;
            rx_read_strobe <= 0;
            rx_buffer <= 0;
        end
    end
endmodule
