


`default_nettype none

module LTC2315_Ctrl #(
    parameter integer BITS     = 14,   // 1 前导 0 + 12 数据 + 1 尾 0
    parameter integer CLK_DIV  = 0,    // SCLK 半周期分频：SCLK = clk/(2*(CLK_DIV+1))
    parameter integer CS_SETUP = 1,    // CS 拉低到第一个 SCLK 的等待周期
    parameter integer CS_HOLD  = 1,    // 末个 SCLK 到 CS 拉高的等待周期
    parameter integer GAP      = 67,   // 两次传输间隔，决定采样率（66 拍 -> 1.515 MSPS）
    parameter integer DUAL     = 1     // 1=两片同时采样  0=单路轮读
) (
    input  wire             clk,        // = clk_samp
    input  wire             rst,


    input  wire             enable,     // 1=连续采样  0=停止（SCLK 停、CS 拉高）
    input  wire [7:0]       clk_div,    // 运行时分频（0 表示用 CLK_DIV 参数）


    output reg              spi_sclk,
    output wire             spi_cs_n,
    output wire             spi_sclk_b, // 与 spi_sclk 同相，接 AD_SCK_1
    output wire             spi_cs_n_b, // 与 spi_cs_n 同相，接 AD_CS_1
    output reg              spi_mosi,   // DUAL=0 时用于通道选择；DUAL=1 时恒 0
    input  wire             spi_miso_a, // AD_SDO_0
    input  wire             spi_miso_b, // AD_SDO_1


    output reg  [15:0]      samp_a,     // A 路样本（16 bit，低 12 位有效：0..4095 offset binary）
    output reg  [15:0]      samp_b,     // B 路样本
    output reg              samp_valid, // 一对 A+B 就绪，单周期脉冲
    output reg  [31:0]      pair_cnt,   // 已成对样本计数（速率观测用）
    output reg  [15:0]      raw_a,      // 最近一次 A 原始码值（调试）
    output reg  [15:0]      raw_b       // 最近一次 B 原始码值（调试）
);


    localparam [4:0] ST_IDLE  = 5'b00001;
    localparam [4:0] ST_SETUP = 5'b00010;
    localparam [4:0] ST_SHIFT = 5'b00100;
    localparam [4:0] ST_HOLD  = 5'b01000;
    localparam [4:0] ST_GAP   = 5'b10000;

    (* fsm_encoding = "one-hot", safe_state = "ST_IDLE" *) reg [4:0] state;
    reg [7:0]  div_cnt;      // SCLK 半周期计数
    reg [7:0]  div_max;      // 当前生效的分频值
    reg [5:0]  bit_cnt;      // 已移入位数
    reg [15:0] sh_a, sh_b;   // 两路移位寄存器
    reg [7:0]  wait_cnt;     // setup/hold/gap 计数
    reg        ch;           // 单路模式下的通道指针：0=A 1=B
    reg        primed;       // LTC2315 首次转换结果无效，后续帧才允许输出

    // 合法态门控：零位/多位/X 编码不得驱动 SPI 物理口
    wire state_valid = (state == ST_IDLE)  || (state == ST_SETUP) ||
                       (state == ST_SHIFT) || (state == ST_HOLD)  ||
                       (state == ST_GAP);

    // 分频：clk_div 为 0 时用参数默认值
    always @(*) begin
        if (clk_div == 8'd0)
            div_max = CLK_DIV[7:0];
        else
            div_max = clk_div;
    end

    // 两路同一时序：不再按通道加额外分频
    wire [7:0] div_max_eff = div_max;


    // 图6 时序（采集中 SCK 保持高 → 2 个前导零）：
    // CS↓ 出第 1 个前导零，第 1 个 SCK↓ 出第 2 个前导零，第 2 个 SCK↓ 出 B11。
    // 共采 14 位（14 个下降沿各采 1 次）：
    //   sh[13]=前导零#2  sh[12]=B11(MSB) ... sh[1]=B0(LSB)  sh[0]=尾零
    // 故 12 位数据域 = sh[12:1]。
    wire [15:0] aligned_a = {sh_a[12:1], 4'b0000};
    wire [15:0] aligned_b = {sh_b[12:1], 4'b0000};
    // 完整 12 位码（右对齐 offset binary，中点 0x800），FIFO 16 位承载
    wire [15:0] code12_a  = {4'b0000, sh_a[12:1]};
    wire [15:0] code12_b  = {4'b0000, sh_b[12:1]};
    reg cs_n_r;                             // 底层 CS（两路共用时序）
    assign spi_sclk_b = spi_sclk;
    assign spi_cs_n   = cs_n_r;             // 两片同时选中，同一 SCLK 采样
    assign spi_cs_n_b = cs_n_r;


    wire pair_done = (DUAL != 0) || (ch == 1'b1);

    always @(posedge clk) begin
        if (rst) begin
            state      <= ST_IDLE;
            spi_sclk   <= 1'b1;   // 图6：采集中 SCK 保持高
            cs_n_r <= 1'b1;
            spi_mosi   <= 1'b0;
            bit_cnt    <= 6'd0;
            sh_a       <= 16'd0;
            sh_b       <= 16'd0;
            div_cnt    <= 8'd0;
            wait_cnt   <= 8'd0;
            ch         <= 1'b0;
            primed     <= 1'b0;
            samp_a     <= 8'd0;
            samp_b     <= 8'd0;
            samp_valid <= 1'b0;
            pair_cnt   <= 32'd0;
            raw_a      <= 16'd0;
            raw_b      <= 16'd0;
        end else if (!state_valid) begin
            // 非法编码：停时钟、释放片选，安全回 IDLE
            state    <= ST_IDLE;
            spi_sclk <= 1'b1;   // 图6：采集中 SCK 保持高
            cs_n_r <= 1'b1;
        end else begin
            samp_valid <= 1'b0;     // 默认单周期脉冲

            case (state)

                ST_IDLE: begin
                    spi_sclk <= 1'b1;   // 图6：采集中 SCK 保持高，其上升沿与 CS 下降沿重合
                    cs_n_r <= 1'b1;
                    if (enable) begin
                        ch       <= 1'b0;
                        wait_cnt <= CS_SETUP[7:0];
                        state    <= ST_SETUP;
                    end
                end


                ST_SETUP: begin
                    cs_n_r <= 1'b0;              // 选中 ADC
                    spi_mosi <= (DUAL != 0) ? 1'b0 : ch; // 单路模式做通道选择
                    if (wait_cnt == 8'd0) begin
                        bit_cnt  <= 6'd0;
                        div_cnt  <= div_max_eff;   // 预置到满值 -> SHIFT 首拍立刻出第一个下降沿
                        sh_a     <= 16'd0;         // 清残留，避免上一帧串位
                        sh_b     <= 16'd0;
                        state    <= ST_SHIFT;
                    end else begin
                        wait_cnt <= wait_cnt - 8'd1;
                    end
                end


                // 移位：SCLK 半周期翻转；上升沿采样 MISO
                // DUAL=1 时 A/B 在同一上升沿采样 -> 零偏斜

                ST_SHIFT: begin
                    if (spi_sclk == 1'b0) begin
                        // ---- 低相位：本位的有效数据区，末尾采样 ----
                        if (div_cnt == div_max_eff) begin
                            div_cnt <= 8'd0;
                            // 采样点距本位下降沿已有半个周期：
                            // CLK_DIV=1（tSCK=40ns）时建立余量 11ns、保持余量 20ns
                            sh_a <= {sh_a[14:0], spi_miso_a};
                            if (DUAL != 0)
                                sh_b <= {sh_b[14:0], spi_miso_b};
                            else
                                sh_b <= {sh_b[14:0], (ch == 1'b1) ? spi_miso_a : 1'b0};
                            if (bit_cnt == BITS[5:0] - 6'd1) begin
                                wait_cnt <= CS_HOLD[7:0];      // 已采满 BITS 位
                                state    <= ST_HOLD;
                            end else begin
                                bit_cnt  <= bit_cnt + 6'd1;
                                spi_sclk <= 1'b1;              // 上升沿
                            end
                        end else
                            div_cnt <= div_cnt + 8'd1;
                    end else begin
                        // ---- 高相位：计数到满值出下降沿 ----
                        if (div_cnt == div_max_eff) begin
                            div_cnt  <= 8'd0;
                            spi_sclk <= 1'b0;                  // 下降沿
                        end else
                            div_cnt <= div_cnt + 8'd1;
                    end
                end


                ST_HOLD: begin
                    if (wait_cnt == 8'd0) begin
                        cs_n_r <= 1'b1;
                        spi_sclk <= 1'b1;   // 图6：SCK 回高，进入下一轮采集

                        if (DUAL != 0) begin

                            raw_a <= aligned_a;
                            raw_b <= aligned_b;
                            if (primed) begin
                                samp_a     <= code12_a;
                                samp_b     <= code12_b;
                                samp_valid <= 1'b1;
                                pair_cnt   <= pair_cnt + 32'd1;
                            end else begin
                                primed <= 1'b1;
                            end
                        end else if (ch == 1'b0) begin

                            raw_a <= aligned_a;
                            samp_a <= code12_a;
                            ch     <= 1'b1;
                        end else begin

                            raw_b <= aligned_b;
                            samp_b <= code12_b;
                            samp_valid <= 1'b1;
                            pair_cnt   <= pair_cnt + 32'd1;
                            ch         <= 1'b0;
                        end

                        wait_cnt <= GAP[7:0];
                        state    <= ST_GAP;
                    end else begin
                        wait_cnt <= wait_cnt - 8'd1;
                    end
                end


                ST_GAP: begin
                    if (wait_cnt == 8'd0) begin
                        if (enable) begin
                            wait_cnt <= CS_SETUP[7:0];
                            state    <= ST_SETUP;
                        end else begin
                            ch    <= 1'b0;
                            state <= ST_IDLE;
                        end
                    end else begin
                        wait_cnt <= wait_cnt - 8'd1;
                    end
                end

                default: state <= ST_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire