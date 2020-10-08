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
    output tx_active,
    output tx_inverted,
    output tx_delay,

    // RX
    input rx,

    output irq
);
    wire xxx_rx;

    SB_IO_OD #(
        .PIN_TYPE(6'b000001),
        .NEG_TRIGGER(1'b0)
    ) rx_io_od (
        .PACKAGEPIN(rx),
        .DIN0(xxx_rx)
    );

    wire xxx_tx_delay;

    SB_IO_OD #(
        .PIN_TYPE(6'b011001),
        .NEG_TRIGGER(1'b0)
    ) tx_delay_io_od (
        .PACKAGEPIN(tx_delay),
        .DOUT0(xxx_tx_delay)
    );

    reg rx_0 = 0;
    reg rx_1 = 0;

    always @(posedge clk)
    begin
        rx_0 <= xxx_rx;
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

    wire loopback;

    wire tx_reset;
    wire internal_tx_active;
    wire internal_tx;
    wire [9:0] tx_data;
    wire tx_load_strobe;
    wire tx_start_strobe;
    wire tx_empty;
    wire tx_full;
    wire tx_ready;

    coax_buffered_tx #(
        .CLOCKS_PER_BIT(16),
        .DEPTH(2048)
    ) coax_buffered_tx (
        .clk(clk),
        .reset(tx_reset),
        .active(internal_tx_active),
        .tx(internal_tx),
        .data(tx_data),
        .load_strobe(tx_load_strobe),
        .start_strobe(tx_start_strobe),
        .empty(tx_empty),
        .full(tx_full),
        .ready(tx_ready)
    );

    wire rx_reset;
    wire rx_active;
    wire rx_error;
    wire [9:0] rx_data;
    wire rx_read_strobe;
    wire rx_empty;

    coax_buffered_rx #(
        .CLOCKS_PER_BIT(16),
        .DEPTH(2048)
    ) coax_buffered_rx (
        .clk(clk),
        .reset(rx_reset),
        .rx(loopback ? internal_tx : (!internal_tx_active ? rx_1 : 0)),
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

        .loopback(loopback),

        .tx_reset(tx_reset),
        .tx_active(internal_tx_active),
        .tx_data(tx_data),
        .tx_load_strobe(tx_load_strobe),
        .tx_start_strobe(tx_start_strobe),
        .tx_empty(tx_empty),
        .tx_full(tx_full),
        .tx_ready(tx_ready),

        .rx_reset(rx_reset),
        .rx_active(rx_active),
        .rx_error(rx_error),
        .rx_data(rx_data),
        .rx_read_strobe(rx_read_strobe),
        .rx_empty(rx_empty)
    );

    coax_tx_distorter #(
        .CLOCKS_PER_BIT(16)
    ) coax_tx_distorter (
        .clk(clk),
        .active_input(!loopback && internal_tx_active),
        .tx_input(internal_tx),
        .active_output(tx_active),
        .tx_delay(xxx_tx_delay),
        .tx_inverted(tx_inverted)
    );

    assign irq = rx_active || rx_error;
endmodule
