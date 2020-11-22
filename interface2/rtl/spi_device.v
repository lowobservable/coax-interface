`default_nettype none

module dual_clock_spi_device (
    input clk_slow,
    input clk_fast,
    input spi_sck,
    input spi_cs,
    input spi_sdi,
    output spi_sdo,
    output reg [7:0] rx_data,
    output reg rx_strobe,
    input [7:0] tx_data,
    input tx_strobe
);
    wire [7:0] rx_data_fast;
    wire rx_strobe_fast;
    reg [7:0] tx_data_fast;
    wire tx_strobe_fast;

    spi_device spi (
        .clk(clk_fast),
        .spi_sck(spi_sck),
        .spi_cs(spi_cs),
        .spi_sdi(spi_sdi),
        .spi_sdo(spi_sdo),
        .rx_data(rx_data_fast),
        .rx_strobe(rx_strobe_fast),
        .tx_data(tx_data_fast),
        .tx_strobe(tx_strobe_fast)
    );

    wire rx_strobe_slow;

    strobe_cdc rx_strobe_cdc (
        .clk_in(clk_fast),
        .strobe_in(rx_strobe_fast),
        .clk_out(clk_slow),
        .strobe_out(rx_strobe_slow)
    );

    strobe_cdc tx_strobe_cdc (
        .clk_in(clk_slow),
        .strobe_in(tx_strobe),
        .clk_out(clk_fast),
        .strobe_out(tx_strobe_fast)
    );

    always @(posedge clk_slow)
    begin
        if (tx_strobe)
            tx_data_fast <= tx_data;

        rx_strobe <= 0;

        if (rx_strobe_slow)
        begin
            rx_data <= rx_data_fast;
            rx_strobe <= 1;
        end
    end
endmodule

module spi_device (
    input clk,
    input spi_sck,
    input spi_cs,
    input spi_sdi,
    output spi_sdo,
    output reg [7:0] rx_data,
    output reg rx_strobe,
    input [7:0] tx_data,
    input tx_strobe
);
    reg [1:0] cs;
    reg [2:0] sck;
    reg [2:0] sdi;

    always @(posedge clk)
    begin
        cs <= { cs[0], spi_cs };

        sck <= { sck[1:0], spi_sck };
        sdi <= { sdi[1:0], spi_sdi };
    end

    reg [3:0] counter;
    reg [7:0] input_data;
    reg [7:0] output_data;

    always @(posedge clk)
    begin
        rx_strobe <= 0;

        if (tx_strobe)
            output_data <= tx_data;

        if (cs[1])
        begin
            counter <= 0;
        end
        else 
        begin
            if (!sck[2] && sck[1])
            begin
                input_data <= { input_data[6:0], sdi[2] };
                counter <= counter + 1;
            end

            if (sck[2] && !sck[1])
            begin
                output_data <= { output_data[6:0], 1'b0 };

                if (counter == 8)
                begin
                    rx_data <= input_data;
                    rx_strobe <= 1;

                    counter <= 0;
                end

            end
        end

    end

    assign spi_sdo = output_data[7];
endmodule
