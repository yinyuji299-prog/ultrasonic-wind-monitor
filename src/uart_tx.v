`timescale 1ns/1ps


`default_nettype none

module uart_tx #(
    parameter integer FIFO_AW = 10,       // 发送 FIFO 深度 = 2^10 = 1024 字节
    parameter integer DIV_W   = 16,       // 分频位宽
    parameter integer DIV_INIT = 868      // 默认 115200 @ 100 MHz
) (
    input  wire             clk,
    input  wire             rst,


    input  wire             tx_push,      // 压入一个字节（单周期）
    input  wire [7:0]       tx_byte,
    input  wire             tx_flush,     // 丢掉 FIFO 里还没发出去的字节
    output wire             tx_full,      // FIFO 满（软核需等）
    output wire             tx_busy,      // 移位器忙

    input  wire [DIV_W-1:0] div_cfg,      // 波特率分频（0 视为默认值）
    input  wire             div_we,       // 写分频寄存器


    output reg              txd
);


    // 字节 FIFO（同步，单时钟）

    localparam FDEPTH = (1 << FIFO_AW);

    (* ram_style = "block" *) reg [7:0] fifo [0:FDEPTH-1];
    reg [FIFO_AW:0] wp, rp;               // 多一位用于区分满/空

    // 经典同步 FIFO 指针：多 1 位 MSB，满 = 地址相同但 MSB 不同


    reg tx_busy_ff;
    assign tx_full    = (wp[FIFO_AW-1:0] == rp[FIFO_AW-1:0]) &&
                        (wp[FIFO_AW]     != rp[FIFO_AW]);
    wire   fifo_empty = (wp == rp);
    wire   pop_fire   = ~fifo_empty && ~tx_busy_ff;

    assign tx_busy = tx_busy_ff;

    always @(posedge clk) begin
        if (rst || tx_flush) begin
            wp <= {(FIFO_AW+1){1'b0}};
            rp <= {(FIFO_AW+1){1'b0}};
        end else begin
            if (tx_push && !tx_full) begin
                fifo[wp[FIFO_AW-1:0]] <= tx_byte;
                wp <= wp + 1'b1;
            end
            if (pop_fire) begin
                rp <= rp + 1'b1;
            end
        end
    end


    // 波特率分频寄存器

    reg [DIV_W-1:0] div_r;
    always @(posedge clk) begin
        if (rst)
            div_r <= DIV_INIT[DIV_W-1:0];
        else if (div_we)
            div_r <= (div_cfg == {DIV_W{1'b0}}) ? DIV_INIT[DIV_W-1:0] : div_cfg;
    end


    // 8N1 发送状态机
    //   每个比特停留 div_r 个时钟；发送顺序：start(0) d0..d7 stop(1)

    localparam S_IDLE = 2'd0;
    localparam S_START= 2'd1;
    localparam S_DATA = 2'd2;
    localparam S_STOP = 2'd3;

    reg [1:0]  st;
    reg [DIV_W-1:0] bcnt;      // 比特内计数
    reg [2:0]  bitidx;         // 数据位索引 0..7
    reg [7:0]  shreg;          // 移位寄存器

    always @(posedge clk) begin
        if (rst) begin
            st        <= S_IDLE;
            txd       <= 1'b1;
            tx_busy_ff<= 1'b0;
            bcnt      <= {DIV_W{1'b0}};
            bitidx    <= 3'd0;
            shreg     <= 8'd0;
        end else begin
            case (st)
                S_IDLE: begin
                    txd        <= 1'b1;
                    tx_busy_ff <= 1'b0;
                    bcnt       <= {DIV_W{1'b0}};
                    bitidx     <= 3'd0;
                    if (!fifo_empty) begin
                        shreg      <= fifo[rp[FIFO_AW-1:0]];
                        tx_busy_ff <= 1'b1;
                        st         <= S_START;
                    end
                end
                S_START: begin
                    txd <= 1'b0;                      // start bit
                    if (bcnt == div_r - 1'b1) begin
                        bcnt <= {DIV_W{1'b0}};
                        st   <= S_DATA;
                        txd  <= shreg[0];             // 先送 d0
                        shreg<= {1'b0, shreg[7:1]};
                        bitidx <= 3'd0;
                    end else
                        bcnt <= bcnt + 1'b1;
                end
                S_DATA: begin
                    if (bcnt == div_r - 1'b1) begin
                        bcnt <= {DIV_W{1'b0}};
                        if (bitidx == 3'd7) begin
                            st  <= S_STOP;
                            txd <= 1'b1;
                        end else begin
                            bitidx <= bitidx + 1'b1;
                            txd    <= shreg[0];
                            shreg  <= {1'b0, shreg[7:1]};
                        end
                    end else
                        bcnt <= bcnt + 1'b1;
                end
                S_STOP: begin
                    txd <= 1'b1;                      // stop bit
                    if (bcnt == div_r - 1'b1) begin
                        bcnt       <= {DIV_W{1'b0}};
                        tx_busy_ff <= 1'b0;
                        st         <= S_IDLE;
                    end else
                        bcnt <= bcnt + 1'b1;
                end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire