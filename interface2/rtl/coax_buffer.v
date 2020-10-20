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

module coax_buffer (
    input clk,
    input reset,
    input [9:0] write_data,
    input write_strobe,
    output [9:0] read_data,
    input read_strobe,
    output empty,
    output almost_full,
    output full
);
    parameter DEPTH = 256;
    parameter ALMOST_FULL_THRESHOLD = 192;

    fifo_sync_ram #(
        .DEPTH(DEPTH),
        .WIDTH(10),
        .ALMOST_FULL_THRESHOLD(ALMOST_FULL_THRESHOLD)
    ) fifo (
        .wr_data(write_data),
        .wr_ena(write_strobe),
        .wr_full(full),
        .rd_data(read_data),
        .rd_ena(read_strobe),
        .rd_empty(empty),
        .clk(clk),
        .rst(reset),
        .almost_full(almost_full)
    );
endmodule
