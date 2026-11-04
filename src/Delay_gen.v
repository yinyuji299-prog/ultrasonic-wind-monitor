`timescale 1ns/1ps


`default_nettype none

module Delay_gen #(
    parameter integer CNT_W = 32
) (
    input  wire               clk,
    input  wire               rst_n,
    input  wire               start_delay,      // 单拍，来自 State_machine
    input  wire [CNT_W-1:0]   delay_cnt,        // 来自 Command_ctrl / APB
    output wire               delay_ack         // 数满 N 拍后拉高一拍
);

    // one-hot：IDLE / COUNT / ACK
    localparam [1:0] S_IDLE  = 2'b01;
    localparam [1:0] S_COUNT = 2'b10;

    (* fsm_encoding = "one-hot", safe_state = "S_IDLE" *) reg [1:0] st;
    reg [CNT_W-1:0] cnt;

    // 0 夹到 1：不能让"写 0"变成无限等待
    wire [CNT_W-1:0] cnt_eff = (delay_cnt == {CNT_W{1'b0}})
                               ? {{CNT_W-1{1'b0}}, 1'b1} : delay_cnt;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st  <= S_IDLE;
            cnt <= {CNT_W{1'b0}};
        end else begin
            case (st)
                S_IDLE: begin
                    cnt <= {CNT_W{1'b0}};
                    if (start_delay) st <= S_COUNT;
                end
                S_COUNT: begin
                    // 数满 cnt_eff 拍后回 IDLE；ack 由组合逻辑当拍给出，
                    // 消费方在当拍末尾的时钟沿采样 → 等待态恰好占 N 拍（无 off-by-one）。
                    if (cnt >= cnt_eff - {{CNT_W-1{1'b0}}, 1'b1})
                        st <= S_IDLE;
                    else
                        cnt <= cnt + 1'b1;
                end
                default: st <= S_IDLE;
            endcase
        end
    end

    // 组合 ack：计数器已注册，比较是纯组合，无抖动风险
    assign delay_ack = (st == S_COUNT) &&
                       (cnt >= cnt_eff - {{CNT_W-1{1'b0}}, 1'b1});

endmodule

`default_nettype wire
