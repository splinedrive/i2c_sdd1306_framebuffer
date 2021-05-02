/*
 *  my_rx_uart - a simple rx uart
 *
 *  copyright (c) 2021  hirosh dabui <hirosh@dabui.de>
 *
 *  permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  the software is provided "as is" and the author disclaims all warranties
 *  with regard to this software including all implied warranties of
 *  merchantability and fitness. in no event shall the author be liable for
 *  any special, direct, indirect, or consequential damages or any damages
 *  whatsoever resulting from loss of use, data or profits, whether in an
 *  action of contract, negligence or other tortious action, arising out of
 *  or in connection with the use or performance of this software.
 *
 */
`timescale 1ns/1ps
`default_nettype none
module my_rx_uart(
           input clk,
           input resetn,
           input rx_in,
           output reg error,
           output reg valid,
           output reg [7:0] rx_data
       );

parameter SYSTEM_CLK_MHZ = 25;
parameter BAUDRATE = 9600;
localparam SYSTEM_CYCLES = $rtoi(SYSTEM_CLK_MHZ*$pow(10,6));
localparam WAITSTATES_BIT_WIDTH = $clog2(SYSTEM_CYCLES);
localparam [WAITSTATES_BIT_WIDTH-1:0] CYCLES_PER_SYMBOL = /*(WAITSTATES_BIT_WIDTH)'*/($rtoi(SYSTEM_CYCLES/BAUDRATE));
localparam [WAITSTATES_BIT_WIDTH-1:0] HALF_CYCLES_PER_SYMBOL = CYCLES_PER_SYMBOL>>1;

initial begin
    $display("SYSTEM_CLK_MHZ:\t\t", SYSTEM_CLK_MHZ);
    $display("SYSTEM_CYCLES:\t\t", SYSTEM_CYCLES);
    $display("BAUDRATE:\t\t", BAUDRATE);
    $display("CYCLES_PER_SYMBOL:\t", CYCLES_PER_SYMBOL);
    $display("WAITSTATES_BIT_WIDTH:\t", WAITSTATES_BIT_WIDTH);
end

reg [2:0] state;
reg [2:0] return_state;
reg [2:0] bit_idx;

reg [WAITSTATES_BIT_WIDTH-1:0] wait_states;

reg [2:0] rx_in_sync;
always @(posedge clk) begin
    if (~resetn) begin
        rx_in_sync <= 0;
    end else begin
        rx_in_sync <= {rx_in_sync[1:0], rx_in};
    end
end

wire start_bit_detected = rx_in_sync[2] & ~rx_in_sync[1]; /* 10 */

always @(posedge clk) begin

    if (~resetn) begin
        state <= 0;
        valid <= 1'b0;
        error <= 1'b0;

        bit_idx <= 0;
        rx_data <= 0;
    end else begin

        case (state)

            0: begin /* idle */
                valid <= 1'b0;
                error <= 1'b0;
                if (start_bit_detected) begin /* high to low */
                    wait_states <= HALF_CYCLES_PER_SYMBOL;
                    return_state <= 1;
                    state <= 7;
                end
            end

            1: begin
                if (~rx_in_sync[2]) begin // still low?
                    wait_states <= CYCLES_PER_SYMBOL;
                    return_state <= 2;
                    state <= 7;
                end else begin
                    state <= 0; // idle
                end
            end

            2: begin
                rx_data[bit_idx] <= rx_in_sync[2]; /* lsb first */
                bit_idx <= bit_idx + 1;

                wait_states <= CYCLES_PER_SYMBOL;
                return_state <= &bit_idx ? 3 : 2;
                state <= 7;
            end

            3: begin
                if (~rx_in_sync[2]) begin // stop bit must high
                    error <= 1'b1;
                    state <= 0;
                end else begin
                    wait_states <= HALF_CYCLES_PER_SYMBOL -1;
                    return_state <= 4;
                    state <= 7;
                end
            end

            4: begin
                valid <= 1'b1;
                state <= 0;
            end

            7: begin /* wait states */
                wait_states <= wait_states -1;
                if (wait_states == 1) begin
                    state <= return_state;
                end
            end

            default: begin
                state <= 0;
            end

        endcase

    end
end /* !reset */

endmodule
