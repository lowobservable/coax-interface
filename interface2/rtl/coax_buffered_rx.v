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

module coax_buffered_rx (
    input clk,
    input reset,
    input rx,
    output active,
    output error,
    output [9:0] data,
    input read_strobe,
    output empty,
    output full
);
    parameter CLOCKS_PER_BIT = 8;
    parameter DEPTH = 256;

    localparam ERROR_OVERFLOW = 10'b0000001000;

    wire coax_rx_error;
    wire [9:0] coax_rx_data;
    wire coax_rx_strobe;

    coax_rx #(
        .CLOCKS_PER_BIT(CLOCKS_PER_BIT)
    ) coax_rx (
        .clk(clk),
        .reset(reset),
        .rx(rx),
        .active(active),
        .error(coax_rx_error),
        .data(coax_rx_data),
        .strobe(coax_rx_strobe)
    );

    wire [9:0] fifo_data;

    // TODO: move this to a coax_buffer module...
    fifo_sync_ram #(
        .DEPTH(DEPTH),
        .WIDTH(10)
    ) fifo (
        .wr_data(coax_rx_data),
        .wr_ena(coax_rx_strobe),
        .wr_full(full),
        .rd_data(fifo_data),
        .rd_ena(read_strobe && !error),
        .rd_empty(empty),
        .clk(clk),
        .rst(reset)
    );

    wire overflow;

    assign overflow = ((active && !previous_active && !empty) || (coax_rx_strobe && full));

    reg overflowed = 0;
    reg previous_active;

    always @(posedge clk)
    begin
        if (reset)
            overflowed <= 0;
        else if (overflow)
            overflowed <= 1;

        previous_active <= active;
    end

    assign error = overflow || overflowed || coax_rx_error;
    assign data = (overflow || overflowed) ? ERROR_OVERFLOW : (coax_rx_error ? coax_rx_data : fifo_data);
endmodule
