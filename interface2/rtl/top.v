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

module top (
    input clk,

    // SPI
    input spi_sck,
    input spi_cs,
    input spi_sdi,
    output spi_sdo,

    // TX

    // RX
    input rx,

    output debug_0,
    output debug_1,
    output debug_2
);
    reg rx_0 = 0;
    reg rx_1 = 0;

    always @(posedge clk)
    begin
        rx_0 <= rx;
        rx_1 <= rx_0;
    end

    wire [7:0] spi_rx_data;
    wire spi_rx_strobe;
    wire [7:0] spi_tx_data;
    wire spi_tx_strobe;

    spi_device spi (
        .clk(clk),
        .reset(/* TODO */ 0),
        .spi_clk(spi_sck),
        .spi_cs(spi_cs),
        .spi_mosi(spi_sdi),
        .spi_miso(spi_sdo),
        .spi_rx_data(spi_rx_data),
        .spi_rx_strobe(spi_rx_strobe),
        .spi_tx_data(spi_tx_data),
        .spi_tx_strobe(spi_tx_strobe)
    );

    /* TODO: TX */

    wire rx_reset;
    wire rx_active;
    wire rx_error;
    wire [9:0] rx_data;
    wire rx_read_strobe;
    wire rx_empty;

    coax_buffered_rx #(
        .CLOCKS_PER_BIT(16),
        .DEPTH(256)
    ) coax_rx (
        .clk(clk),
        .reset(rx_reset),
        .rx(/* TODO: rx_enable ? (loopback ? tx : rx_1) : 0 */ rx_1),
        .active(rx_active),
        .error(rx_error),
        .data(rx_data),
        .read_strobe(rx_read_strobe),
        .empty(rx_empty)
    );

    control control (
        .clk(clk),
        .reset(/* TODO */ 0),

        .spi_cs(spi_cs),
        .spi_rx_data(spi_rx_data),
        .spi_rx_strobe(spi_rx_strobe),
        .spi_tx_data(spi_tx_data),
        .spi_tx_strobe(spi_tx_strobe),

        /* TODO: tx... */

        .rx_reset(rx_reset),
        .rx_active(rx_active),
        .rx_error(rx_error),
        .rx_data(rx_data),
        .rx_read_strobe(rx_read_strobe),
        .rx_empty(rx_empty)
    );

    assign debug_0 = 1;
    assign debug_1 = 1;
    assign debug_2 = 1;
endmodule
