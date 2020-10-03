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

module coax_buffered_tx (
    input clk,
    input reset,
    output active,
    output tx,
    input [9:0] data,
    input load_strobe,
    input start_strobe,
    output empty,
    output full
);
    parameter CLOCKS_PER_BIT = 8;
    parameter DEPTH = 256;

    localparam STATE_IDLE = 0;
    localparam STATE_TRANSMITTING = 1;

    reg state = STATE_IDLE;
    reg next_state;

    wire [9:0] coax_tx_data;
    wire coax_tx_strobe;
    wire coax_tx_ready;

    coax_tx #(
        .CLOCKS_PER_BIT(CLOCKS_PER_BIT)
    ) coax_tx (
        .clk(clk),
        .reset(reset),
        .active(active),
        .tx(tx),
        .data(coax_tx_data),
        .strobe(coax_tx_strobe),
        .ready(coax_tx_ready)
    );

    reg fifo_read_strobe = 0;
    reg next_fifo_read_strobe;

    // TODO: move this to a coax_buffer module...
    fifo_sync_ram #(
        .DEPTH(DEPTH),
        .WIDTH(10)
    ) fifo (
        .wr_data(data),
        .wr_ena(load_strobe),
        .wr_full(full),
        .rd_data(coax_tx_data),
        .rd_ena(fifo_read_strobe),
        .rd_empty(empty),
        .clk(clk),
        .rst(reset)
    );

    assign coax_tx_strobe = (state == STATE_TRANSMITTING && !empty && coax_tx_ready);

    always @(*)
    begin
        next_state = state;

        // FIFO read strobe is delayed 1 clock from TX load.
        next_fifo_read_strobe = coax_tx_strobe;

        case (state)
            STATE_IDLE:
            begin
                if (start_strobe && !empty)
                    next_state = STATE_TRANSMITTING;
            end

            STATE_TRANSMITTING:
            begin
                if (empty && !active)
                    next_state = STATE_IDLE;
            end
        endcase
    end

    always @(posedge clk)
    begin
        state <= next_state;

        fifo_read_strobe <= next_fifo_read_strobe;

        if (reset)
        begin
            state <= STATE_IDLE;

            fifo_read_strobe <= 0;
        end
    end
endmodule
