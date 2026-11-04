`timescale 1ns/1ps


`default_nettype none

module uart_rx #(
    parameter integer DIV_W   = 16,
    parameter integer DIV_INIT = 868      // 115200 @ 100 MHz
) (
    input  wire                 clk,
    input  wire                 rst,

    input  wire [DIV_W-1:0]     div_cfg,  // 波特率分频（0 = 用 DIV_INIT）
    input  wire                 enable,   // 0 = 关闭（静默，不产生任何事件）

    input  wire                 rxd,      // 物理输入（屏 TX）

    output reg  [7:0]           rx_byte,
    output reg                  rx_valid, // 收到一个字节（单周期脉冲）
    output reg                  rx_err    // 帧错误：停止位不是 1
);


    reg rxd_s1, rxd_s2, rxd_s3;
    always @(posedge clk) begin
        if (rst) begin
            rxd_s1 <= 1'b1; rxd_s2 <= 1'b1; rxd_s3 <= 1'b1;
        end else begin
            rxd_s1 <= rxd;
            rxd_s2 <= rxd_s1;
            rxd_s3 <= rxd_s2;
        end
    end
    wire rxd_fall = (rxd_s3 & ~rxd_s2);       // 起始位下降沿


    wire [DIV_W-1:0] div_eff = (div_cfg == 0) ? DIV_INIT[DIV_W-1:0] : div_cfg;
    wire [DIV_W-1:0] div_half = {1'b0, div_eff[DIV_W-1:1]};   // 半位：在中点采样

    reg [DIV_W-1:0] baud_cnt;
    reg             baud_tick;
    always @(posedge clk) begin
        if (rst) begin
            baud_cnt  <= {DIV_W{1'b0}};
            baud_tick <= 1'b0;
        end else if (baud_cnt == div_eff) begin
            baud_cnt  <= {DIV_W{1'b0}};
            baud_tick <= 1'b1;
        end else begin
            baud_cnt  <= baud_cnt + 1'b1;
            baud_tick <= 1'b0;
        end
    end


    localparam [1:0] S_IDLE = 2'd0,
                     S_START= 2'd1,
                     S_DATA = 2'd2,
                     S_STOP = 2'd3;

    reg [1:0]  state;
    reg [2:0]  bit_idx;
    reg [7:0]  shift;
    reg [DIV_W-1:0] wait_cnt;

    always @(posedge clk) begin
        if (rst) begin
            state    <= S_IDLE;
            bit_idx  <= 3'd0;
            shift    <= 8'd0;
            wait_cnt <= {DIV_W{1'b0}};
            rx_byte  <= 8'd0;
            rx_valid <= 1'b0;
            rx_err   <= 1'b0;
        end else begin
            rx_valid <= 1'b0;
            rx_err   <= 1'b0;
            if (!enable) begin
                state   <= S_IDLE;
                bit_idx <= 3'd0;
                wait_cnt<= {DIV_W{1'b0}};
            end else begin
                case (state)
                    S_IDLE: begin
                        if (rxd_fall) begin
                            wait_cnt <= div_half;      // 等到起始位中点
                            state    <= S_START;
                        end
                    end
                    S_START: begin
                        if (wait_cnt == 0) begin
                            if (rxd_s3 == 1'b0) begin              
                                wait_cnt <= div_eff;
                                state    <= S_DATA;
                                bit_idx  <= 3'd0;
                            end else begin
                                state <= S_IDLE;       // 毛刺，丢弃
                            end
                        end else begin
                            wait_cnt <= wait_cnt - 1'b1;
                        end
                    end
                    S_DATA: begin
                        if (baud_tick) begin
                            shift   <= {rxd_s3, shift[7:1]};   // LSB first
                            wait_cnt<= div_eff;
                            if (bit_idx == 3'd7) begin
                                state <= S_STOP;
                            end
                            bit_idx <= bit_idx + 1'b1;
                        end
                    end
                    S_STOP: begin
                        if (baud_tick) begin
                            rx_byte  <= shift;
                            rx_valid <= 1'b1;
                            rx_err   <= (rxd_s3 != 1'b1);      // 停止位应为 1
                            state    <= S_IDLE;
                        end
                    end
                    default: state <= S_IDLE;
                endcase
            end
        end
    end

endmodule

`default_nettype wire
