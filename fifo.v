/*
 *  fifo.v
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

module fifo
       #(parameter DATA_WIDTH=8,
         parameter DEPTH=4)
       (
           input clk,
           input resetn,
           input [DATA_WIDTH-1:0] din,
           output [DATA_WIDTH-1:0] dout,
           input push,
           input pop,
           output full,
           output empty
       );

reg [DATA_WIDTH-1:0] fifo[0:DEPTH-1];

reg [$clog2(DEPTH):0] cnt;
reg [$clog2(DEPTH)-1:0] rd_ptr;
reg [$clog2(DEPTH)-1:0] wr_ptr;

assign empty = cnt == 0;
assign full = cnt == DEPTH;

always @(posedge clk) begin
    if (~resetn) begin
        rd_ptr <= 0;
        wr_ptr <= 0;
        cnt <= 0;
    end else begin
        if (push) begin
            fifo[wr_ptr] <= din;
            wr_ptr <= wr_ptr + 1;

            if (!pop || empty) cnt <= cnt + 1;
        end

        if (pop) begin
            rd_ptr <= rd_ptr + 1;

            if (!push || full) cnt <= cnt - 1;
        end

    end
end

assign dout = fifo[rd_ptr];

endmodule
