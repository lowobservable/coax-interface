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

module coax_rx (
    input clk,
    input reset,
    input rx,
    output reg active,
    output reg error,
    output reg [9:0] data,
    output reg strobe = 0,
    input parity
);
    parameter CLOCKS_PER_BIT = 8;

    localparam ERROR_LOSS_OF_MID_BIT_TRANSITION = 10'b0000000001;
    localparam ERROR_PARITY = 10'b0000000010;
    localparam ERROR_INVALID_END_SEQUENCE = 10'b0000000100;

    localparam STATE_IDLE = 0;
    localparam STATE_START_SEQUENCE_1 = 1;
    localparam STATE_START_SEQUENCE_2 = 2;
    localparam STATE_START_SEQUENCE_3 = 3;
    localparam STATE_START_SEQUENCE_4 = 4;
    localparam STATE_START_SEQUENCE_5 = 5;
    localparam STATE_START_SEQUENCE_6 = 6;
    localparam STATE_START_SEQUENCE_7 = 7;
    localparam STATE_START_SEQUENCE_8 = 8;
    localparam STATE_START_SEQUENCE_9 = 9;
    localparam STATE_SYNC_BIT = 10;
    localparam STATE_DATA_BIT = 11;
    localparam STATE_PARITY_BIT = 12;
    localparam STATE_END_SEQUENCE_1 = 13;
    localparam STATE_END_SEQUENCE_2 = 14;
    localparam STATE_ERROR = 15;

    reg [3:0] state = STATE_IDLE;
    reg [3:0] next_state;
    reg [3:0] previous_state;
    reg [7:0] state_counter;
    reg [7:0] next_state_counter;

    reg previous_rx;

    reg bit_timer_reset = 0;
    reg next_bit_timer_reset;

    reg [9:0] next_data;
    reg next_strobe;

    reg [9:0] input_data;
    reg [9:0] next_input_data;
    reg [3:0] bit_counter = 0;
    reg [3:0] next_bit_counter;

    reg next_active;
    reg next_error;

    wire sample;
    wire synchronized;

    coax_rx_bit_timer #(
        .CLOCKS_PER_BIT(CLOCKS_PER_BIT)
    ) bit_timer (
        .clk(clk),
        .rx(rx),
        .reset(bit_timer_reset),
        .sample(sample),
        .synchronized(synchronized)
    );

    always @(*)
    begin
        next_state = state;
        next_state_counter = state_counter + 1;

        next_bit_timer_reset = 0;

        next_data = data;
        next_strobe = 0;

        next_input_data = input_data;
        next_bit_counter = bit_counter;

        case (state)
            STATE_IDLE:
            begin
                next_bit_timer_reset = 1;

                if (!rx && previous_rx)
                begin
                    next_state = STATE_START_SEQUENCE_1;
                    next_state_counter = 0;
                end
            end

            STATE_START_SEQUENCE_1:
            begin
                if (sample)
                begin
                    if (synchronized && rx)
                        next_state = STATE_START_SEQUENCE_2;
                    else
                        next_state = STATE_IDLE;

                    next_state_counter = 0;
                end
                else if (state_counter >= (CLOCKS_PER_BIT * 2))
                begin
                    next_state = STATE_IDLE;
                    next_state_counter = 0;
                end
            end

            STATE_START_SEQUENCE_2:
            begin
                if (sample)
                begin
                    if (synchronized && rx)
                        next_state = STATE_START_SEQUENCE_3;
                    else
                        next_state = STATE_IDLE;

                    next_state_counter = 0;
                end
            end

            STATE_START_SEQUENCE_3:
            begin
                if (sample)
                begin
                    if (synchronized && rx)
                        next_state = STATE_START_SEQUENCE_4;
                    else
                        next_state = STATE_IDLE;

                    next_state_counter = 0;
                end
            end

            STATE_START_SEQUENCE_4:
            begin
                if (sample)
                begin
                    if (synchronized && rx)
                        next_state = STATE_START_SEQUENCE_5;
                    else
                        next_state = STATE_IDLE;

                    next_state_counter = 0;
                end
            end

            STATE_START_SEQUENCE_5:
            begin
                if (sample)
                begin
                    if (synchronized && rx)
                        next_state = STATE_START_SEQUENCE_6;
                    else
                        next_state = STATE_IDLE;

                    next_state_counter = 0;
                end
            end

            STATE_START_SEQUENCE_6:
            begin
                if (!rx)
                begin
                    next_state = STATE_START_SEQUENCE_7;
                    next_state_counter = 0;
                end
                else if (state_counter >= CLOCKS_PER_BIT)
                begin
                    next_state = STATE_IDLE;
                    next_state_counter = 0;
                end
            end

            STATE_START_SEQUENCE_7:
            begin
                if (rx)
                begin
                    next_state = STATE_START_SEQUENCE_8;
                    next_state_counter = 0;
                end
                else if (state_counter >= (CLOCKS_PER_BIT * 2))
                begin
                    next_state = STATE_IDLE;
                    next_state_counter = 0;
                end
            end

            STATE_START_SEQUENCE_8:
            begin
                if (!rx)
                begin
                    next_bit_timer_reset = 1;
                    next_state = STATE_START_SEQUENCE_9;
                    next_state_counter = 0;
                end
                else if (state_counter >= (CLOCKS_PER_BIT * 2))
                begin
                    next_state = STATE_IDLE;
                    next_state_counter = 0;
                end
            end

            STATE_START_SEQUENCE_9:
            begin
                // This is really the first STATE_SYNC_BIT but we treat it
                // differently and consider it part of the start
                // sequence.

                if (sample && synchronized)
                begin
                    if (rx)
                    begin
                        next_bit_counter = 0;
                        next_state = STATE_DATA_BIT;
                    end
                    else
                    begin
                        next_state = STATE_IDLE;
                    end

                    next_state_counter = 0;
                end
                else if (state_counter >= CLOCKS_PER_BIT)
                begin
                    next_state = STATE_IDLE;
                    next_state_counter = 0;
                end
           end

           STATE_SYNC_BIT:
           begin
               if (sample)
               begin
                   if (synchronized)
                   begin
                       if (rx)
                       begin
                           next_bit_counter = 0;
                           next_state = STATE_DATA_BIT;
                       end
                       else
                       begin
                           next_state = STATE_END_SEQUENCE_1;
                       end

                       next_state_counter = 0;
                   end
                   else
                   begin
                       next_data = ERROR_LOSS_OF_MID_BIT_TRANSITION;
                       next_state = STATE_ERROR;
                       next_state_counter = 0;
                   end
               end
           end

           STATE_DATA_BIT:
           begin
               if (sample)
               begin
                   if (synchronized)
                   begin
                       next_input_data = { input_data[8:0], rx };

                       if (bit_counter < 9)
                       begin
                           next_bit_counter = bit_counter + 1;
                       end
                       else
                       begin
                           next_state = STATE_PARITY_BIT;
                       end

                       next_state_counter = 0;
                   end
                   else
                   begin
                       next_data = ERROR_LOSS_OF_MID_BIT_TRANSITION;
                       next_state = STATE_ERROR;
                       next_state_counter = 0;
                   end
               end
           end

           STATE_PARITY_BIT:
           begin
               if (sample)
               begin
                   if (synchronized)
                   begin
                       // Parity includes the sync bit.
                       if (rx == (parity == 1 ? ^{ 1'b1, input_data } : ~^{ 1'b1, input_data }))
                       begin
                           next_strobe = 1;
                           next_data = input_data;
                           next_state = STATE_SYNC_BIT;
                       end
                       else
                       begin
                           next_data = ERROR_PARITY;
                           next_state = STATE_ERROR;
                       end

                       next_state_counter = 0;
                   end
                   else
                   begin
                       next_data = ERROR_LOSS_OF_MID_BIT_TRANSITION;
                       next_state = STATE_ERROR;
                       next_state_counter = 0;
                   end
               end
           end

           STATE_END_SEQUENCE_1:
           begin
               if (rx)
               begin
                   next_state = STATE_END_SEQUENCE_2;
                   next_state_counter = 0;
               end
               else if (state_counter >= CLOCKS_PER_BIT)
               begin
                   next_data = ERROR_INVALID_END_SEQUENCE;
                   next_state = STATE_ERROR;
                   next_state_counter = 0;
               end
           end

           STATE_END_SEQUENCE_2:
           begin
               // TODO: should this go to ERROR on timeout?
               if (!rx)
               begin
                   next_state = STATE_IDLE;
                   next_state_counter = 0;
               end
               else if (state_counter >= (CLOCKS_PER_BIT * 2))
               begin
                   next_state = STATE_IDLE;
                   next_state_counter = 0;
               end
           end
        endcase
    end

    always @(posedge clk)
    begin
        state <= next_state;
        state_counter <= next_state_counter;

        bit_timer_reset <= next_bit_timer_reset;

        data <= next_data;
        strobe <= next_strobe;

        input_data <= next_input_data;
        bit_counter <= next_bit_counter;

        active <= (state >= STATE_SYNC_BIT && state <= STATE_PARITY_BIT);
        error <= (state == STATE_ERROR);

        if (reset)
        begin
            bit_timer_reset <= 1;

            state <= STATE_IDLE;
            state_counter <= 0;

            data <= 10'b0000000000;
            strobe <= 0;

            input_data <= 10'b0000000000;
            bit_counter <= 0;

            active <= 0;
            error <= 0;
        end

        previous_rx <= rx;
        previous_state <= state;
    end
endmodule
