`timescale 1 ns/10 ps
`default_nettype none
`include "i2c_api.vh"
`ifdef SIM
module top_tb;
inout  sda;
`else
/*
module top(input CLK,
inout P2_1,
output P2_2);
// these are the led holding registers, whatever you write to these appears on the led display
wire clk = CLK;
wire scl_pin = P2_1;
wire sda = P2_2;
*/
module top(input clk,
           inout sda,
           output scl_pin,
           output uart_tx,
           input uart_rx);
reg [7:0] leds_a[3:0];
`endif


wire rx_in = uart_rx;
assign uart_tx = rx_in;


localparam TRANSFER_RATE   = 1_000_000;
localparam CLK_FREQ        = 25_000_000;
localparam SYSTEM_CLK_MHZ  = 25;
localparam CLK_PERIOD      = 1/$itor(CLK_FREQ);
localparam PERIOD_NS       = $rtoi(CLK_PERIOD*10.0e9);
localparam SCL_CYCLES      = CLK_FREQ / TRANSFER_RATE;
localparam BAUDRATE        = 115200;

reg push;
reg pop;
reg [7:0] din;
wire [7:0] dout;
wire empty;
wire full;

fifo#(.DATA_WIDTH(8),
      .DEPTH(4))
    fifo_i( .clk(clk),
            .resetn(resetn),
            .push(push),
            .pop(pop),
            .din(din),
            .dout(dout),
            .empty(empty),
            .full(full),
          );


wire [7:0] data_rx;
wire i2c_ready;
wire stopped;
wire sda_oe;
`ifdef SIM
reg sda_in;
`else
wire sda_in;
`endif
wire sda_out;
wire scl;
reg enable;

wire error;
wire valid;
wire [7:0] rx_data;
wire rx_valid;

my_rx_uart #(SYSTEM_CLK_MHZ, BAUDRATE) my_rx_uart_i(.clk(clk),
           .resetn(resetn), .rx_in(rx_in),
           .error(error), .valid(rx_valid), .rx_data(rx_data));

always @(posedge clk) begin
    if (~resetn) begin
        push <= 0;
    end else begin

        if (rx_valid & ~full) begin
            din <= rx_data;
            push <= 1'b1;
        end else begin
            push <= 1'b0;
        end

    end

end

i2c_api
    #(
        .TRANSFER_RATE(TRANSFER_RATE),
        .CLK_FREQ(CLK_FREQ)
    ) i2c_api_i (
        .clk(clk),
        .resetn(resetn),
        .enable(enable),
        .slave_addr(slave_addr),
        .device_register(device_register),
        .data_tx(mux_data_tx),
        .data_rx(data_rx),
        .scl(scl),
        .sda_oe(sda_oe),
        .i2c_function(mux_i2c_function),
`ifdef SIM
        .sda(sda),
`endif
        .sda_in(sda_in),
        .sda_out(sda_out),
        .done(done),
        .ready(i2c_ready)
    );

`ifndef SIM
SB_IO #(
          .PIN_TYPE(6'b1010_01),
          .PULLUP(1'b0)
      ) sda_i (
          .PACKAGE_PIN(sda),
          .OUTPUT_ENABLE(sda_oe),
          .D_OUT_0(sda_out),
          .D_IN_0(sda_in)
      );

SB_IO #(
          .PIN_TYPE(6'b1010_01),
          .PULLUP(1'b0)
      ) sdc_i (
          .PACKAGE_PIN(scl_pin),
          .OUTPUT_ENABLE(1'b1),
          .D_OUT_0(scl)
      );
`endif


`ifdef SIM
reg clk = 0;
always #(PERIOD_NS>>1) clk = ~clk;

initial begin
    $dumpfile("testbench.vcd");
    $dumpvars(0, top_tb);
    $dumpon;
end

initial begin
    sda_in = 1'b0;
end
`endif

reg [5:0] reset_cnt = 0;
wire resetn = &reset_cnt;

always @(posedge clk) begin
    reset_cnt <= reset_cnt + !resetn;
end

/* oled part */
localparam SETCONTRAST         = 8'h81;
localparam DISPLAYALLON_RESUME = 8'hA4;
localparam DISPLAYALLON        = 8'hA5;
localparam NORMALDISPLAY       = 8'hA6;
localparam INVERTDISPLAY       = 8'hA7;
localparam DISPLAYOFF          = 8'hAE;
localparam DISPLAYON           = 8'hAF;
localparam SETDISPLAYOFFSET    = 8'hD3;
localparam SETCOMPINS          = 8'hDA;
localparam SETVCOMDETECT       = 8'hDB;
localparam SETDISPLAYCLOCKDIV  = 8'hD5;
localparam SETPRECHARGE        = 8'hD9;
localparam SETMULTIPLEX        = 8'hA8;
localparam SETLOWCOLUMN        = 8'h00;
localparam SETHIGHCOLUMN       = 8'h10;
localparam SETSTARTLINE        = 8'h40;
localparam MEMORYMODE          = 8'h20;
localparam COLUMNADDR          = 8'h21;
localparam PAGEADDR            = 8'h22;
localparam COMSCANINC          = 8'hC0;
localparam COMSCANDEC          = 8'hC8;
localparam SEGREMAP            = 8'hA0;
localparam CHARGEPUMP          = 8'h8D;


reg [7:0] font;
reg [3:0] x; // 0 - 15
reg [1:0] y; // 0 - 3
reg  [9:0] ssd1306_addr;
wire [7:0] ssd1306_out;
wire ready_gfx;
wire done_gfx;
reg render;
reg rd;
reg and_fb;

gfx_unit gfx_unit_i(
             .clk(clk),
             .and_fb(and_fb),
             .font(font),
             .x(x), // 0 - 15
             .y(y), // 0 - 3
             .ssd1306_addr(ssd1306_addr),
             .ssd1306_out(ssd1306_out),
             .rd(rd),
             .ready(ready_gfx),
             .resetn(resetn),
             .render(render),
             .done(done_gfx)
         );


reg [4:0] state;
reg [3:0] return_state;
reg [31:0] wait_states;
wire done;
wire [7:0] data_tx;

`I2C_API_DECLS

    /* instructions */
    localparam NOP      = 3'd00;
localparam STOP     = 3'd01;
localparam JMP      = 3'd02;
localparam LD       = 3'd03;
localparam WAIT     = 3'd04;
localparam JNZ      = 3'd05;
localparam FB_FLUSH = 3'd06;
localparam PUTC     = 3'd07;

wire [7:0] i2c_function;
wire [6:0] slave_addr;
wire [7:0] device_register;
wire [2:0] opcode;
wire [7:0] operand;

reg [3:0] led_idx;

reg [7:0] ip;

reg [ 8 + 7 + 8 + 8 + 3 +  8 - 1:0] ctrl;
always @(*) begin
    case (ip)
        /* i2c_function                  slave_addr   out                  reg     opcode   operand */
        /* DISPLAYOFF */
        00: ctrl <= {I2C_NOP,             7'h3c,       8'h00,                8'h00,  NOP,      8'd00};
        01: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        02: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        03: ctrl <= {I2C_WRITE_RAW,       7'h3c,       DISPLAYOFF,           8'h00,  NOP,      8'h00};
        04: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        05: ctrl <= {I2C_NOP,             7'h3c,       8'h00,                8'h00,  NOP,      8'd00};
        /* SETDISPLAYCLOCKDIV 0x80 */
        06: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        07: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        08: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETDISPLAYCLOCKDIV,   8'h00,  NOP,      8'h00};
        09: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h80,                8'h00,  NOP,      8'h00};
        10: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* SETMULTIPLEX 0x3f */
        11: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        12: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        13: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETMULTIPLEX,         8'h00,  NOP,      8'h00};
        14: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h3f,                8'h00,  NOP,      8'h00};
        15: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};


        /* SETDISPLAYOFFSET 0x0 */
        16: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        17: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        18: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETDISPLAYOFFSET,     8'h00,  NOP,      8'h00};
        19: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        20: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* SETSTARTLINE | 0 */
        21: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        21: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        22: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETSTARTLINE | 1'b0,  8'h00,  NOP,      8'h00};
        23: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* CHARGEPUMP 0x14 */
        24: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        25: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        26: ctrl <= {I2C_WRITE_RAW,       7'h3c,       CHARGEPUMP,           8'h00,  NOP,      8'h00};
        27: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h14,                8'h00,  NOP,      8'h00};
        28: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* MEMORYMODE 0x0*/
        29: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        30: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        31: ctrl <= {I2C_WRITE_RAW,       7'h3c,       MEMORYMODE,           8'h00,  NOP,      8'h00};
        32: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        33: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* SEGREMAP | 0x1 */
        34: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        35: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        36: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SEGREMAP | 1'b1,      8'h00,  NOP,      8'h00};
        37: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* COMSCANDEC */
        38: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        39: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        40: ctrl <= {I2C_WRITE_RAW,       7'h3c,       COMSCANDEC,           8'h00,  NOP,      8'h00};
        41: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* SETCOMPINS 0x12 */
        42: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        43: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        44: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETCOMPINS,           8'h00,  NOP,      8'h00};
        45: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h12,                8'h00,  NOP,      8'h00};
        46: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* SETCONTRAST 0xcf*/
        47: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        48: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        49: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETCONTRAST,          8'h00,  NOP,      8'h00};
        50: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'hcf,                8'h00,  NOP,      8'h00};
        51: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* SETVCOMDETECT 0x40 */
        52: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        53: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        54: ctrl <= {I2C_WRITE_RAW,       7'h3c,       SETVCOMDETECT,        8'h00,  NOP,      8'h00};
        55: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h40,                8'h00,  NOP,      8'h00};
        56: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* DISPLAYALLON_RESUME */
        57: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        58: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        59: ctrl <= {I2C_WRITE_RAW,       7'h3c,       DISPLAYALLON_RESUME,  8'h00,  NOP,      8'h00};
        60: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* NORMALDISPLAY */
        61: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        62: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        63: ctrl <= {I2C_WRITE_RAW,       7'h3c,       NORMALDISPLAY,        8'h00,  NOP,      8'h00};
        64: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* DISPLAYON */
        65: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        66: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        67: ctrl <= {I2C_WRITE_RAW,       7'h3c,       DISPLAYON,            8'h00,  NOP,      8'h00};
        68: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'd165};

        /* nop */
        69: ctrl <= {I2C_NOP,              7'h00,       8'h00,               8'h00,  JMP,     8'd79};

        70: ctrl <= {I2C_NOP,              7'h68,       8'h00,               8'h00,  NOP,      8'h00};
        /* read seconds */
        71: ctrl <= {I2C_READ_U8,          7'h68,       8'h00,               8'h00,  LD,       8'h03};
        /* read minutes */
        72: ctrl <= {I2C_READ_U8,          7'h68,       8'h00,               8'h01,  LD,       8'h02};
        /* read hours   */
        73: ctrl <= {I2C_READ_U8,          7'h68,       8'h00,               8'h02,  LD,       8'h01};
        /* */
        74: ctrl <= {I2C_READ_U8,          7'h68,       8'h00,               8'h02,  JMP,      8'd79};
        /* write seconds */
        75: ctrl <= {I2C_WRITE_8,          7'h68,       8'h00,               8'h00,  NOP,      8'h00};
        /* write minutes */
        76: ctrl <= {I2C_WRITE_8,          7'h68,       8'h01,               8'h01,  NOP,      8'h00};
        /* write hours   */
        77: ctrl <= {I2C_WRITE_8,          7'h68,       8'h02,               8'h02,  NOP,      8'h05};
        /* wait */
        78: ctrl <= {I2C_NOP    ,          7'h68,       8'h00,               8'h00,  NOP,     8'hff};

        /* COLUMNADDR */
        79: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        80: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        81: ctrl <= {I2C_WRITE_RAW,       7'h3c,       COLUMNADDR,           8'h00,  NOP,      8'h00};
        82: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* 0 */
        83: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        84: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        85: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        86: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* 127 */
        87: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        88: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        89: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h7f,                8'h00,  NOP,      8'h00};
        90: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* PAGEADDR */
        91: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        92: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        93: ctrl <= {I2C_WRITE_RAW,       7'h3c,       PAGEADDR,             8'h00,  NOP,      8'h00};
        94: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* 0 */
        95: ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        96: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        97: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        98: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        /* 7 */
        99:  ctrl <= {I2C_START,           7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        100: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h00,                8'h00,  NOP,      8'h00};
        101: ctrl <= {I2C_WRITE_RAW,       7'h3c,       8'h07,                8'h00,  NOP,      8'h00};
        102: ctrl <= {I2C_STOP,            7'h3c,       8'h00,                8'h00,  NOP,      8'h00};

        103: ctrl <= {I2C_START,         7'h3c,       8'h00,                 8'h00,  NOP,       8'd69};
        104: ctrl <= {I2C_WRITE_RAW,     7'h3c,       8'h40,                 8'h00,  FB_FLUSH,  8'd69};
        105: ctrl <= {I2C_STOP,          7'h3c,       8'h00,                 8'h00,  NOP,       8'd69};
        106: ctrl <= {I2C_NOP,           7'h3c,       8'h00,                 8'h00,  NOP,       8'd69};

        107: ctrl <= {I2C_NOP,           7'h3c,       8'h00,                 8'h00,  PUTC,      8'd00};
        108: ctrl <= {I2C_NOP,           7'h3c,       8'h00,                 8'h00,  NOP,       8'd00};
        109: ctrl <= {I2C_NOP,           7'h3c,       8'h00,                 8'h00,  JMP,       8'd79};

        default:
            ctrl <= 0;
    endcase
end


wire [7:0] mux_data_tx = opcode == FB_FLUSH ? (rd ? ssd1306_out : 8'h40) : data_tx;
wire [7:0] mux_i2c_function = opcode == FB_FLUSH & rd ? I2C_WRITE_RAW : i2c_function;

/*       8               7           8          8              3         8 */
assign {i2c_function, slave_addr, data_tx, device_register, opcode, operand} = ctrl;

reg [7:0] i2c_function_fb_flush;
always @(posedge clk) begin
    if (~resetn) begin
        render <= 0;
        x <= ~0;
        y <= ~0;
        font <= 0;
        state <= 0;
        return_state <= 0;
        ip <= 0;
        enable <= 1'b1;
        led_idx <= 0;
        and_fb <= 1'b0;
        rd <= 1'b0;

`ifndef SIM
        leds_a[0] <= ~0;
        leds_a[1] <= ~0;
        leds_a[2] <= ~0;
        leds_a[3] <= ~0;
`endif
    end else begin
        case (state)
            0: begin
                if (i2c_ready) begin
                    state <= 1;
                end
            end
            1: begin
                if (done) begin

                    case (opcode)
                        FB_FLUSH:begin
                            state <= 10;
                        end
                        JMP: begin
                            ip <= operand;
                            state <= 0;
                        end
                        LD: begin
                            led_idx <= operand;
                            ip <= ip + 1;
                            state <= 0;
                        end
                        WAIT: begin
                          `ifndef SIM
                            wait_states[22 -:8] <= operand -1;
                            return_state <= 0;
                            state <= 15;
                          `else
                            state <= 0;
                          `endif
                            ip <= ip + 1;
                        end
                        PUTC: begin
                            if (~empty) begin
                                x <= x + 1;
                                if (&x) y <= y + 1;
                                rd <= 0;
                                font <= 8'hff;
                                and_fb <= 1'b1;
                                render <= 1'b1;
                                if (ready_gfx) begin
                                    state <= 5;
                                end
                                else begin
                                    ip <= ip + 1;
                                    state <= 0;
                                end
                            end
                        end
                        STOP: begin
                          `ifdef SIM
                            $finish;
                    `endif
                            ip <= ip;
                            state <= 0;
                        end
                        default: begin
                            ip <= ip + 1;
                            state <= 0;
                        end
                    endcase
                `ifndef SIM
                    if (opcode == LD) begin
                        leds_a[operand] <= ~data_rx;
                        state <= 0;
                    end
                `endif
                end
            end

            5: begin
                render <= 1'b0;
                if (done_gfx) begin
                    and_fb <= 1'b0;
                    pop <= 1'b1;
                    font <= dout;
                    state <= 6;
                end
            end

            6: begin
                pop <= 1'b0;
                if (ready_gfx) begin
                    render <= 1'b1;
                    state <= 7;
                end
            end

            7: begin
                if (done_gfx) begin
                    render <= 1'b0;
                    ip <= ip + 1;
                    state <= 0;
                end
            end


            10: begin

                if (i2c_ready) begin
                    ssd1306_addr <= 0;
                    rd <= 1'b1;
                    state <= 11;
                end
            end

            11: begin
                if (done)begin
                    state <= 2;
                end
            end

            2: begin
                if (&ssd1306_addr) begin
                    ip <= ip + 1;
                    state <= 0;
                    rd <= 1'b0;
                end else begin
                    if (i2c_ready) begin
                        ssd1306_addr <= ssd1306_addr + 1;
                        state <= 3;
                    end
                end

            end

            3: begin
                if (done) begin
                    state <= 2;
                end
            end

            15: begin
                wait_states <= wait_states -1;
                if (wait_states == 1) state <= return_state;
            end

            default:
                state <= 0;
        endcase
    end
end

endmodule
