`timescale 1ns/1ps
`default_nettype none

// DAC7811 控制器：协议沿用“同时激发 pwm”基线。
// 首次启动发送 0x9000 关闭菊链，随后连续发送 {4'b0001, da_code}。
// 数据帧为 16 bit、4 个 clk/bit，并保留 2 个 clk 的帧间隔，
// 因而稳态帧周期约 66 个 100 MHz clk，约 1.515 MSPS。
module DAC7811_Ctrl #(
    parameter integer CLK_DIV  = 0,
    parameter [11:0]  DAC_CODE = 12'd2048
) (
    input  wire        clk,
    input  wire        rst_n,
    input  wire        start_ADDA,    // 采集窗口电平：1=连续更新，0=停止
    input  wire [11:0] da_code,
    output reg         sync_n,
    output reg         sclk,
    output reg         sdin,
    output reg         da_ack
);
    localparam [15:0] FRAME_DAISY_OFF = 16'b1001_0000_0000_0000;

    localparam [1:0] ST_IDLE  = 2'b01;
    localparam [1:0] ST_SHIFT = 2'b10;
    localparam [1:0] ST_GAP   = 2'b11;

    (* fsm_encoding = "one-hot", safe_state = "ST_IDLE" *) reg [1:0] st;
    reg        frame_sel;             // 0=菊链关闭帧，1=数据帧
    reg [15:0] sh;
    reg [4:0]  bit_cnt;
    reg [1:0]  ph;                    // 4 个 clk/bit
    reg [1:0]  gap_cnt;               // 帧尾/帧间隔
    reg        gap_phase;             // 0=帧尾，1=帧间隔
    reg        stop_pending;           // 已检测采集结束，正在发送 4095 归零帧
    reg        daisy_done;             // 0x9000 菊链关闭帧已发过（只发一次）

    wire [15:0] frame_load = {4'b0001, da_code};
    wire [15:0] frame_zero = {4'b0001, 12'hFFF};
    wire        st_ok = (st == ST_IDLE) || (st == ST_SHIFT) || (st == ST_GAP);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= ST_IDLE;
            frame_sel <= 1'b0;
            sh        <= 16'd0;
            bit_cnt   <= 5'd0;
            ph        <= 2'd0;
            gap_cnt   <= 2'd0;
            gap_phase    <= 1'b0;
            stop_pending <= 1'b0;
            daisy_done   <= 1'b0;
            sync_n       <= 1'b1;
            sclk      <= 1'b0;
            sdin      <= 1'b0;
            da_ack    <= 1'b0;
        end else if (!st_ok) begin
            st     <= ST_IDLE;
            sync_n <= 1'b1;
            sclk   <= 1'b0;
            da_ack <= 1'b0;
        end else begin
            da_ack <= 1'b0;

            case (st)
                ST_IDLE: begin
                    sync_n <= 1'b1;
                    sclk   <= 1'b0;
                    if (start_ADDA) begin
                        bit_cnt      <= 5'd0;
                        ph           <= 2'd0;
                        stop_pending <= 1'b0;
                        sync_n       <= 1'b0;
                        if (!daisy_done) begin
                            // 上电后第一轮：先发 0x9000 关闭菊链。
                            // 这一帧数据域是 0x000（0 V / 最小增益），
                            // 只能发一次，不能每轮都发，否则每轮窗口
                            // 开头 0.67 us 的 VGA 增益都是错的。
                            frame_sel <= 1'b0;
                            sh        <= FRAME_DAISY_OFF;
                            sdin      <= FRAME_DAISY_OFF[15];
                        end else begin
                            // 之后每轮直接进增益帧，第一个采样点就是放大后的。
                            frame_sel <= 1'b1;
                            sh        <= frame_load;
                            sdin      <= frame_load[15];
                        end
                        st <= ST_SHIFT;
                    end
                end

                ST_SHIFT: begin
                    sclk <= (ph >= 2'd2);
                    if (ph == 2'd3) begin
                        // 下降沿采样当前 SDIN，协议与基线一致。
                        ph      <= 2'd0;
                        bit_cnt <= bit_cnt + 5'd1;
                        if (bit_cnt == 5'd15) begin
                            gap_cnt   <= 2'd1;
                            gap_phase <= 1'b0;
                            st        <= ST_GAP;
                        end
                    end else if (ph == 2'd1 && bit_cnt != 5'd0) begin
                        // 下降沿后换下一位，避免与采样沿同拍改变 SDIN。
                        ph   <= 2'd2;
                        sh   <= {sh[14:0], 1'b0};
                        sdin <= sh[14];
                    end else begin
                        ph <= ph + 2'd1;
                    end
                end

                ST_GAP: begin
                    sclk <= 1'b0;
                    if (gap_cnt != 2'd0) begin
                        gap_cnt <= gap_cnt - 2'd1;
                    end else if (!gap_phase) begin
                        // SYNC 上升沿使当前帧生效。
                        sync_n <= 1'b1;
                        if (!frame_sel) begin
                            // 0x9000 菊链关闭帧完成：以后各轮不再发它。
                            daisy_done <= 1'b1;
                            frame_sel  <= 1'b1;
                            sh         <= frame_load;
                            sdin       <= frame_load[15];
                            bit_cnt    <= 5'd0;
                            ph         <= 2'd0;
                            gap_phase  <= 1'b1;
                            st         <= ST_GAP;
                        end else if (start_ADDA) begin
                            // 采集仍在进行，保持约 66 clk/帧的连续更新。
                            sh           <= frame_load;
                            stop_pending <= 1'b0;
                            sdin      <= frame_load[15];
                            bit_cnt   <= 5'd0;
                            ph        <= 2'd0;
                            gap_phase <= 1'b1;
                            st        <= ST_GAP;
                        end else if (!stop_pending) begin
                            // 采集结束：先发送一次窗口外归零码，再退出。
                            stop_pending <= 1'b1;
                            sh           <= frame_zero;
                            sdin         <= frame_zero[15];
                            bit_cnt      <= 5'd0;
                            ph           <= 2'd0;
                            gap_phase    <= 1'b1;
                            st           <= ST_GAP;
                        end else begin
                            stop_pending <= 1'b0;
                            da_ack       <= 1'b1;
                            st            <= ST_IDLE;
                        end
                    end else begin
                        // SYNC 保持高一个 clk 后开始下一数据帧。
                        sync_n    <= 1'b0;
                        gap_phase <= 1'b0;
                        st        <= ST_SHIFT;
                    end
                end

                default: st <= ST_IDLE;
            endcase
        end
    end
endmodule

`default_nettype wire
