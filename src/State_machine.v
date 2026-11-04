`timescale 1ns/1ps

`default_nettype none

//======================================================================
// State_machine —— 一帧测量的主控
//
// 一轮时序（以 PWM 发射起点为 0）：
//   0        FIRE   双路同时发射 6 个 200 kHz 周期，约 30 us
//   30us     WAIT   Delay_gen 延时到采集窗口起点
//   192us    ACQ    ADC 使能，两路 FIFO 同时写入
//   满 256 点 DRAIN  等软核把两路都读空
//   读空     GAP     1 ms 硬件间隔
//   1ms 后   IDLE    等下一次触发
//
// 超时保护：任一非空闲态停留超过 timeout 即进入 S_HALT，
//           此时 acq_en=0、不再触发 PWM，直到重新 start。
//======================================================================
module State_machine #(
    parameter [31:0] GAP_CYC = 32'd100000      // 1 ms @ 100 MHz
) (
    input  wire        clk,
    input  wire        rst_n,

    input  wire        start,                  // APB 写 PWM_CTRL 的上升沿
    input  wire [31:0] timeout,                // 每状态超时门限（clk 数）

    output reg         start_pwm,              // 单拍脉冲：启动 PWM 突发
    input  wire        pwm_ack,                // PWM 突发结束

    output reg         start_delay1,           // 单拍脉冲：启动 330 us 延时
    input  wire        delay1_ack,             // 延时到

    input  wire        fifo_full,              // 当前通道 FIFO 已满
    input  wire        read_all_done,          // 软核已把 FIFO 读空
    input  wire        fifo_arm_ack,           // 两路 FIFO 均已完成 arm
    input  wire        fifo_release_ack,       // 两路 FIFO 均已完成 release

    output reg         fifo_arm,               // 单拍脉冲：清指针、开新帧
    output reg         fifo_release,           // 单拍脉冲：放行状态机

    output reg         acq_en,                 // 采集窗口使能（电平，仅 ACQ 态为 1）

    output wire        busy,
    output reg         done,
    output reg         error,
    output reg  [7:0]  error_code,
    output wire [2:0]  state_dbg
);

    //------------------------------------------------------------------
    // 状态编码
    //------------------------------------------------------------------
    localparam [2:0] S_IDLE  = 3'd0;
    localparam [2:0] S_FIRE  = 3'd1;
    localparam [2:0] S_WAIT  = 3'd2;
    localparam [2:0] S_ACQ   = 3'd3;
    localparam [2:0] S_DRAIN = 3'd4;
    localparam [2:0] S_GAP   = 3'd5;
    localparam [2:0] S_ARM   = 3'd6;
    localparam [2:0] S_HALT  = 3'd7;   // 超时安全态：PWM/ADC/DAC 全关

    reg [2:0] state;
    reg [2:0] next_state;

    //------------------------------------------------------------------
    // 内部寄存器
    //------------------------------------------------------------------
    reg        start_d;                        // start 打一拍，用于取上升沿
    reg [31:0] gap_cnt;                        // 轮间 1 ms 倒计时
    reg [31:0] wt_cnt;                         // 状态停留计数（超时保护）
    reg        release_seen;                   // release ACK 可早于 1ms，先锁存

    wire start_rise = start & ~start_d;
    wire gap_busy   = (gap_cnt != 32'd0);
    wire fifo_done  = (state == S_DRAIN) & read_all_done;
    wire release_ready = release_seen | fifo_release_ack;
    wire release_done = (state == S_GAP) & ~gap_busy & release_ready;
    // 采集窗口只在 330 us 延时完成后打开，避免 FIFO/DA 提前启动。
    wire acq_start = (state == S_WAIT) & delay1_ack;
    // 超时：强制回安全态（关闭 ADC/DAC/PWM），不容许错误态继续跑。
    wire timeout_hit = (state != S_IDLE) &&
                       (timeout != 32'd0) &&
                       (wt_cnt >= (timeout - 32'd1));

    //==================================================================
    // 一段：状态寄存器
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) state <= S_IDLE;
        else        state <= next_state;
    end

    //==================================================================
    // 二段：次态组合逻辑（不含任何输出赋值）
    //==================================================================
    always @(*) begin
        case (state)
            S_IDLE  : next_state = (start_rise & ~gap_busy) ? S_FIRE  : S_IDLE;
            S_FIRE  : next_state = pwm_ack                  ? S_WAIT  : S_FIRE;
            S_WAIT  : next_state = delay1_ack             ? S_ARM   : S_WAIT;
            S_ARM   : next_state = fifo_arm_ack            ? S_ACQ   : S_ARM;
            S_ACQ   : next_state = fifo_full               ? S_DRAIN : S_ACQ;
            S_DRAIN : next_state = read_all_done                  ? S_GAP   : S_DRAIN;
            S_GAP   : next_state = (!gap_busy && release_ready) ? S_IDLE : S_GAP;
            // 超时：任何非空闲态一律强制进安全态，不再继续本轮流程。
            S_HALT  : next_state = start_rise ? S_IDLE : S_HALT;
            default : next_state = S_IDLE;
        endcase
        // 最高优先级：超时覆盖一切次态判断
        if (timeout_hit) next_state = S_HALT;
    end

    //==================================================================
    // 三段：输出与计数器（全部寄存，与 clk 严格对齐）
    //==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            start_d      <= 1'b0;
            start_pwm    <= 1'b0;
            start_delay1 <= 1'b0;
            fifo_arm     <= 1'b0;
            fifo_release <= 1'b0;
            acq_en       <= 1'b0;
            done         <= 1'b0;
            error        <= 1'b0;
            error_code   <= 8'd0;
            gap_cnt      <= 32'd0;
            wt_cnt       <= 32'd0;
            release_seen <= 1'b0;
        end else begin

            start_d <= start;

            // ---- release ACK 可能早于 1 ms 结束：锁存至 GAP 完成 ----
            if (state != S_GAP)
                release_seen <= 1'b0;
            else if (fifo_release_ack)
                release_seen <= 1'b1;
            else if (!gap_busy && release_ready)
                release_seen <= 1'b0;

            // ---- 单拍脉冲：只在触发条件成立的那一拍为 1 ----
            start_pwm    <= (state == S_IDLE) & start_rise & ~gap_busy;
            // 延时到点才 arm；该脉冲与采集窗口开始同一状态边界。
            fifo_arm     <= acq_start;
            start_delay1 <= (state == S_FIRE) & pwm_ack;
            fifo_release <= fifo_done;
            done         <= fifo_done;

            // ---- 电平输出：只由当前状态决定 ----
            acq_en <= (state == S_ACQ);

            // ---- 轮间 1 ms 硬件延时 ----
            if (fifo_done)
                gap_cnt <= GAP_CYC;
            else if (gap_busy)
                gap_cnt <= gap_cnt - 32'd1;

            // ---- 状态停留计数（超时保护）----
            if (state == S_IDLE || state == S_HALT) wt_cnt <= 32'd0;
            else                                    wt_cnt <= wt_cnt + 32'd1;

            // ---- 超时：锁存错误码，下一次触发时清 ----
            if (timeout_hit) begin
                error      <= 1'b1;
                error_code <= {5'd0, state};
            end else if ((state == S_IDLE) && start_rise) begin
                error      <= 1'b0;
                error_code <= 8'd0;
            end

        end
    end

    assign busy      = (state != S_IDLE) | gap_busy;
    assign state_dbg = state;

endmodule

`default_nettype wire
