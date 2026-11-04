`timescale 1ns/1ps


`default_nettype none

module PWM_gen #(
    parameter integer PER_W    = 16,
    parameter integer PER_INIT = 500,      // 200 kHz @ 100 MHz
    parameter integer CYC_INIT = 8
) (
    input  wire             clk,
    input  wire             rst_n,

    input  wire             start_pwm,     // 单拍请求，来自 State_machine
    input  wire [PER_W-1:0] freq_pwm,      // 周期分频（clk 数），来自 Command_ctrl/APB
    input  wire [15:0]      pwm_cycles,    // 脉冲个数（整周期数）

    output wire             pwm_out1,      // A换能器，空闲恒 0
    output wire             pwm_out2,      // B换能器，空闲恒 0
    output reg              pwm_ack,       // 脉冲串发完，单拍回 State_machine
    output wire             burst_active   // 调试：正在发脉冲
);


    localparam [2:0] S_IDLE = 3'b001;
    localparam [2:0] S_TX   = 3'b010;
    localparam [2:0] S_ACK  = 3'b100;

    (* fsm_encoding = "one-hot", safe_state = "S_IDLE" *) reg [2:0] st;
    reg [PER_W-1:0] half_cnt;
    reg [15:0]      cyc_cnt;
    reg             phase;

    // 参数保护：0 用默认值；半周期下限 2（防 per_eff=1 时 half-1 下溢）
    wire [PER_W-1:0] per_eff = (freq_pwm == {PER_W{1'b0}})
                               ? PER_INIT[PER_W-1:0] : freq_pwm;
    wire [15:0]      cyc_eff = (pwm_cycles == 16'd0) ? CYC_INIT[15:0] : pwm_cycles;
    wire [PER_W-1:0] half_raw = per_eff >> 1;
    wire [PER_W-1:0] half_eff = (half_raw < 16'd2) ? 16'd2 : half_raw;

    // 合法态门控：零位/多位/X 都不得认为在发
    wire st_valid = (st == S_IDLE) || (st == S_TX) || (st == S_ACK);

    // 两路同源：A、B 同时发射同一段猝发
    reg  pwm_out1_r;
    wire pwm_tx = st_valid && (st == S_TX);
    assign pwm_out1 = pwm_tx ? pwm_out1_r : 1'b0;
    assign pwm_out2 = pwm_tx ? pwm_out1_r : 1'b0;
    assign burst_active = pwm_tx;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // 复位期间严格低电平
            st       <= S_IDLE;
            half_cnt <= {PER_W{1'b0}};
            cyc_cnt  <= 16'd0;
            phase    <= 1'b0;
            pwm_ack  <= 1'b0;
            pwm_out1_r <= 1'b0;
        end else if (!st_valid) begin
            // 非法编码：停发，安全回 IDLE，输出清零
            st       <= S_IDLE;
            phase    <= 1'b0;
            pwm_ack  <= 1'b0;
            pwm_out1_r <= 1'b0;
        end else begin
            pwm_ack <= 1'b0;
            case (st)

                S_IDLE: begin
                    // 空闲：输出每个时钟都强制 0，绝不残留高电平
                    phase    <= 1'b0;
                    half_cnt <= {PER_W{1'b0}};
                    cyc_cnt  <= 16'd0;
                    pwm_out1_r <= 1'b0;
                    if (start_pwm) begin
                        phase    <= 1'b1;       // 从高半周开始
                        pwm_out1_r <= 1'b1;     // 串首当拍出高电平
                        st       <= S_TX;
                    end
                end


                // 发脉冲：每 half_eff 拍翻一次输出；凑满"高+低"记一个整周期
                // 10 个周期 = 20 个半周期，方波严格 50%

                S_TX: begin
                    if (half_cnt == half_eff - {{PER_W-1{1'b0}}, 1'b1}) begin
                        half_cnt <= {PER_W{1'b0}};
                        if (phase == 1'b1) begin
                            // 高半周结束 → 转低半周
                            phase    <= 1'b0;
                            pwm_out1_r <= 1'b0;
                        end else begin
                            // 低半周结束 → 一个整周期完成
                            if (cyc_cnt == cyc_eff - 16'd1) begin
                                // 最后一个整周期：强制低，进 ACK 回 ack
                                pwm_out1_r <= 1'b0;
                                st       <= S_ACK;
                            end else begin
                                cyc_cnt  <= cyc_cnt + 16'd1;
                                phase    <= 1'b1;
                                pwm_out1_r <= 1'b1; // 下一个高半周
                            end
                        end
                    end else begin
                        half_cnt <= half_cnt + 1'b1;
                    end
                end


                S_ACK: begin
                    pwm_out1_r <= 1'b0;       // 保证低
                    pwm_ack  <= 1'b1;
                    st       <= S_IDLE;
                end

                default: begin
                    st       <= S_IDLE;
                    phase    <= 1'b0;
                    pwm_out1_r <= 1'b0;
                end
            endcase
        end
    end

endmodule

`default_nettype wire
