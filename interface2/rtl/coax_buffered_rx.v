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
    output full,
    input parity
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
        .strobe(coax_rx_strobe),
        .parity(parity)
    );

    wire [9:0] coax_buffer_data;

    coax_buffer #(
        .DEPTH(DEPTH)
    ) coax_buffer (
        .clk(clk),
        .reset(reset),
        .write_data(coax_rx_data),
        .write_strobe(coax_rx_strobe),
        .read_data(coax_buffer_data),
        .read_strobe(read_strobe && !error),
        .empty(empty),
        .full(full)
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
    assign data = (overflow || overflowed) ? ERROR_OVERFLOW : (coax_rx_error ? coax_rx_data : (empty ? 10'b0 : coax_buffer_data));
endmodule
