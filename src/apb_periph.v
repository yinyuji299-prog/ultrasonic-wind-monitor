`timescale 1ns/1ps


`default_nettype none

module apb_periph #(
    parameter integer N     = 512,
    parameter integer NW    = 9,
    parameter integer ADDR_W= 9
) (
    input  wire         clk_sys,
    input  wire         rst_sys,
    input  wire         clk_samp,
    input  wire         rst_samp,


    input  wire [15:0]  PADDR,
    input  wire         PSEL,
    input  wire         PENABLE,
    input  wire         PWRITE,
    input  wire [31:0]  PWDATA,
    output wire         PREADY,
    output wire [31:0]  PRDATA,
    output wire         PSLVERROR,


    input  wire         samp_valid,
    input  wire [15:0]  samp_a,
    input  wire [15:0]  samp_b,
    output wire         frame_ready,     // 给 PWM_gen：A/B 两路都收满才关窗


    output wire         screen_txd,
    input  wire         screen_rxd,      // 屏 TX → FPGA（D5/GPIOL_28）


    output wire         pwm_out1,
    output wire         pwm_out2,
    // DAC7811 物理口（两路，由本模块内部 DAC7811_Ctrl 驱动；


    output wire         da_sync_0_o,
    output wire         da_sclk_0_o,
    output wire         da_sdin_0_o,
    output wire         da_sync_1_o,
    output wire         da_sclk_1_o,
    output wire         da_sdin_1_o,
    output wire         meas_busy,       // State_machine 忙（给顶层 LED/调试）
    output wire         acq_en_o         // 采集窗口使能（给 ADC 的 enable）
);

    assign PREADY    = 1'b1;
    assign PSLVERROR = 1'b0;

    wire apb_wr = PSEL & PENABLE & PWRITE;
    wire apb_rd = PSEL & PENABLE & ~PWRITE;


    // 1. 控制寄存器

    reg        fifo_win_gate;                                          
    reg [8:0]  fifo_rd_addr_a;        // A 路读指针（独立）
    reg [8:0]  fifo_rd_addr_b;        // B 路读指针（独立）
    reg        fifo_rd_inc;


    reg [3:0]  dbg_bdelay;            // 复位默认 0 = 直通
    reg [15:0] bdly [0:15];           // 16 深 × 8 bit 移位链（综合成 SRL/FF）
    integer    bi;
    reg        scr_flush;
    reg        scr_cle;
    reg        scr_purge;
    reg        scr_abort;
    reg        scr_pt_we;
    reg [7:0]  scr_pt_data;
    reg        scr_pt_clr;
    reg        scr_fld_we;
    reg [7:0]  scr_fld_id;
    reg [1:0]  scr_fld_type;
    reg        scr_fld_sign;
    reg [23:0] scr_fld_val;
    reg [1:0]  scr_curve_ch;
    reg [1:0]  scr_curve_obj;      /* 2026-11-01 曲线控件号 0/1/2 = s0/s1/s2 */
    reg        uart_push;
    reg [7:0]  uart_byte;
    reg [15:0] uart_div;
    reg        uart_div_we;
    reg        start_pwm;      // 一次测量请求（送 State_machine）
    reg [31:0] freq_pwm;       // PWM 周期分频（32 位与 State_machine 对齐）
    reg [15:0] pwm_cycles;     // 脉冲个数（16 位够用：最大 65535 个）
    reg [31:0] delay_1;        // 采前：回波传播等待 T_prop
    reg [11:0] da_code;        // DAC7811 增益码（每轮重写）


    reg [31:0] timeout_cfg;
    reg        pwm_winclr;                                   
    reg        rd_done_a, rd_done_b;   // 读指针绕回产生的一拍脉冲（该路读完）


    reg        rd_a_seen, rd_b_seen;
    reg        dsp_done_p;             // 软核写 MEAS_DSP bit0 产生的一拍脉冲
    reg [15:0] dsp_done_cnt;                                     


    wire        fsm_fifo_arm_req, fsm_fifo_release_req, fsm_acq_en;
    wire        fsm_error, fsm_done;
    wire [7:0]  fsm_error_code8;
    wire [2:0]  fsm_state_3;
    wire [31:0] fsm_freq_pwm, fsm_delay_1;
    wire [15:0] fsm_pwm_cycles;
    wire [11:0] fsm_da_code;
    wire        pwm_ack_w, delay1_ack_w, da_ack_0, da_ack_1;
    wire        start_pwm_w, start_delay1_w;
    wire        pwm_burst_active;
    reg [31:0] fld_snapshot;          // 最近写入的数值域（回读用）


    reg        scr_rx_en;                            
    reg [15:0] scr_rx_div;
    reg        rx_st_a5;
    reg [7:0]  rx_last;

    wire        urx_valid;
    wire [7:0]  urx_byte;
    wire        t_fe, t_fd, t_a5, t_lastv;
    wire [7:0]  t_last;

    uart_rx #(
        .DIV_W   (16),
        .DIV_INIT(868)                 // 115200 @ 100 MHz
    ) u_uart_rx (
        .clk      (clk_sys),
        .rst      (rst_sys),
        .div_cfg  (scr_rx_div),
        .enable   (scr_rx_en),
        .rxd      (screen_rxd),
        .rx_byte  (urx_byte),
        .rx_valid (urx_valid),
        .rx_err   ()
    );

    tjc_rx u_tjc_rx (
        .clk        (clk_sys),
        .rst        (rst_sys),
        .enable     (scr_rx_en),
        .clr_cmd    (apb_rd && (PADDR[7:2] == 6'h2D)),
        .byte_in    (urx_byte),
        .byte_valid (urx_valid),
        .got_fe     (t_fe),
        .got_fd     (t_fd),
        .got_a5     (t_a5),
        .last_byte  (t_last),
        .last_valid (t_lastv)
    );


    wire        apb_wr_hit = apb_wr;                             
    wire [5:0]  wr_reg_sel = PADDR[7:2];           // 写地址译码

    // 各路写脉冲（组合）
    wire w_ctrl  = apb_wr_hit && (wr_reg_sel == 6'h10);   // SCR_CTRL  0x40
    wire w_pt    = apb_wr_hit && (wr_reg_sel == 6'h12);   // SCR_PT    0x48
    wire w_ptclr = apb_wr_hit && (wr_reg_sel == 6'h13);   // SCR_PTCLR 0x4C
    wire w_fld   = apb_wr_hit && (wr_reg_sel == 6'h14);   // SCR_FIELD 0x50
    wire w_utx   = apb_wr_hit && (wr_reg_sel == 6'h20);   // UART_TX   0x80
    wire w_rxctrl= apb_wr_hit && (wr_reg_sel == 6'h2C);   // SCR_RX_CTRL 0xB0
    wire w_udiv  = apb_wr_hit && (wr_reg_sel == 6'h22);   // UART_DIV  0x88
    wire w_pwm   = apb_wr_hit && (wr_reg_sel == 6'h24);   // PWM_CTRL  0x90
    wire w_wincl = apb_wr_hit && (wr_reg_sel == 6'h29);   // PWM_WINCLR 0xA4（清 error）
    wire w_dly2  = apb_wr_hit && (wr_reg_sel == 6'h06);   // MEAS_DLY2 0x18
    wire w_dacd  = apb_wr_hit && (wr_reg_sel == 6'h07);   // DA_CODE   0x1C
    wire w_dsp   = apb_wr_hit && (wr_reg_sel == 6'h08);   // MEAS_DSP  0x20
    wire w_tmo1  = apb_wr_hit && (wr_reg_sel == 6'h0C);   // TIMEOUT_ALL 0x30
    wire w_tmop  = apb_wr_hit && (wr_reg_sel == 6'h0D);   // TIMEOUT_PWM 0x34
    wire w_tmod  = apb_wr_hit && (wr_reg_sel == 6'h0E);   // TIMEOUT_DLY1 0x38
    wire w_tmoa  = apb_wr_hit && (wr_reg_sel == 6'h0F);   // TIMEOUT_ADDA 0x3C


    wire w_abort = w_ctrl && PWDATA[3];


    always @(posedge clk_sys) begin
        if (rst_sys) begin
            fifo_win_gate<= 1'b1;                               
            fifo_rd_addr_a <= 9'd0;
            fifo_rd_addr_b <= 9'd0;
            fifo_rd_inc  <= 1'b0;
            scr_flush    <= 1'b0;
            scr_cle      <= 1'b0;
            scr_purge    <= 1'b0;
            scr_abort    <= 1'b0;
            scr_pt_we    <= 1'b0;
            scr_pt_clr   <= 1'b0;
            scr_fld_we   <= 1'b0;
            uart_push    <= 1'b0;
            uart_div_we  <= 1'b0;
            start_pwm    <= 1'b0;
            pwm_winclr   <= 1'b0;
            rd_done_a    <= 1'b0;
            rd_done_b    <= 1'b0;
            rd_a_seen    <= 1'b0;
            rd_b_seen    <= 1'b0;
            dsp_done_p   <= 1'b0;
            dsp_done_cnt <= 16'd0;
            scr_pt_data  <= 8'd0;
            scr_fld_id   <= 8'd0;
            scr_fld_type <= 2'd0;
            scr_fld_sign <= 1'b0;
            scr_fld_val  <= 24'd0;
            scr_curve_ch  <= 2'd0;
            scr_curve_obj <= 2'd0;
            uart_byte    <= 8'd0;
            uart_div     <= 16'd868;      // 115200 @ 100 MHz
            freq_pwm     <= 32'd500;     // 200 kHz @ 100 MHz
            pwm_cycles   <= 16'd10;      // 分时规格：A/B 每轮 10 个 200 kHz 周期
            delay_1      <= 32'd15700;   // 2026-10-28 L 122->80mm，回波提前 122us；窗口起点 157+30=187us
            da_code      <= 12'd2048;    // 中等增益
            timeout_cfg  <= 32'd2000000;      // 20 ms @100 MHz
            fld_snapshot <= 32'd0;
            scr_rx_en    <= 1'b1;    // 默认开启，官方握手(0x88/0xFE/0xFD)需要它
            dbg_bdelay   <= 4'd0;                                                
            scr_rx_div   <= 16'd868;
        end else begin


            scr_flush  <= w_ctrl & PWDATA[0];
            scr_cle    <= w_ctrl & PWDATA[1];
            scr_purge  <= w_ctrl & PWDATA[2];
            scr_abort  <= w_abort;
            scr_pt_we  <= w_pt;
            scr_pt_clr <= w_ptclr;
            scr_fld_we <= w_fld;
            uart_push  <= w_utx;
            uart_div_we<= w_udiv;
            // 单拍脉冲：组合脉冲再打一拍（同 scr_pt_we 的写法）。

            //   那种写法会被 efx_map 吃掉 CE，D 直连 PWDATA。
            start_pwm  <= w_pwm & PWDATA[0];
            pwm_winclr <= w_wincl;

            // 两个来源合并成一次翻转，避免同一个时钟内双重非阻塞赋值。


            //   由调用方保证不会同拍发两次 arm。）

            if (apb_wr) begin
                case (PADDR[7:2])
                    6'h00: begin                       // FIFO_CTRL
                        if (PWDATA[1]) begin           // flush：两路读指针同时归零
                            fifo_rd_addr_a <= 9'd0;
                            fifo_rd_addr_b <= 9'd0;
                        end
                        fifo_win_gate <= PWDATA[2];             // 0=自由采（SDG 调试）1=窗口
                    end
                    6'h12: scr_pt_data <= PWDATA[7:0];  // SCR_PT 数据；写使能已在上面锁存
                    6'h14: begin                            // SCR_FIELD(0x50) 数据；写使能已锁存
                        scr_fld_id   <= PWDATA[4:0];
                        scr_fld_type <= PWDATA[6:5];
                        scr_fld_sign <= PWDATA[7];
                        scr_fld_val  <= PWDATA[31:8];
                        fld_snapshot <= PWDATA;
                    end
                    6'h16: begin scr_curve_ch  <= PWDATA[9:8];    // SCR_CURVE(0x58) 通道
                                scr_curve_obj <= PWDATA[11:10]; end  // 控件号
                    6'h20: uart_byte <= PWDATA[7:0];        // UART_TX 数据；写使能已锁存
                    6'h22: uart_div  <= PWDATA[15:0];       // UART_DIV；div_we 已锁存


                    6'h25: freq_pwm <= PWDATA;            // PWM_PERIOD(0x94)
                    6'h26: pwm_cycles <= PWDATA[15:0];      // PWM_CYCLES(0x98)
                    6'h27: delay_1  <= PWDATA[31:0];      // PWM_DELAY(0x9C) = Delay_1
                    6'h07: da_code  <= PWDATA[11:0];      // DA_CODE(0x1C)
                    6'h08: dsp_done_p <= w_dsp & PWDATA[0]; // MEAS_DSP(0x20) bit0=DSP 跑完


                    6'h0C: timeout_cfg <= PWDATA;   // TIMEOUT_ALL(0x30)：统一超时
                    6'h0D: timeout_cfg <= PWDATA;              
                    6'h0E: timeout_cfg <= PWDATA;
                    6'h0F: timeout_cfg <= PWDATA;

                    6'h2A: dbg_bdelay <= PWDATA[3:0];       // DBG_DELAY(0xA8)
                    6'h2C: begin                            // SCR_RX_CTRL(0xB0)
                        scr_rx_en  <= w_rxctrl | PWDATA[0];
                        if (PWDATA[31:16] != 16'd0) scr_rx_div <= PWDATA[31:16];
                    end
                    default: ;
                endcase
            end


            // 指针绕回 511→0 时产生一拍 read_done：软核读满 512 个即该路读完，
            // 不必额外写"读完成"寄存器（忘了写也不会卡住状态机，只会走 delay_2 超时）
            rd_done_a <= 1'b0;
            rd_done_b <= 1'b0;
            if (apb_rd && (PADDR[7:2] == 6'h02)) begin
                if (fifo_rd_addr_a == (N[8:0] - 9'd1)) begin
                    fifo_rd_addr_a <= 9'd0;
                    rd_done_a      <= 1'b1;
                end else
                    fifo_rd_addr_a <= fifo_rd_addr_a + 9'd1;
            end
            if (apb_rd && (PADDR[7:2] == 6'h05)) begin
                if (fifo_rd_addr_b == (N[8:0] - 9'd1)) begin
                    fifo_rd_addr_b <= 9'd0;
                    rd_done_b      <= 1'b1;
                end else
                    fifo_rd_addr_b <= fifo_rd_addr_b + 9'd1;
            end


            if (dsp_done_p) dsp_done_cnt <= dsp_done_cnt + 16'd1;


            if (fsm_state_3 == 3'd4) begin                // S_DRAIN：等软核读
                if (rd_done_a) rd_a_seen <= 1'b1;
                if (rd_done_b) rd_b_seen <= 1'b1;
            end else begin
                rd_a_seen <= 1'b0;
                rd_b_seen <= 1'b0;
            end

        end
    end


    wire [15:0] ff_rdata_a, ff_rdata_b;
    wire        ff_ready_a, ff_ready_b;
    wire        ff_busy_a, ff_busy_b;
    wire [15:0] ff_wcount_a, ff_wcount_b;
    wire        ff_arm_ack_a, ff_arm_ack_b;
    wire        ff_rel_ack_a, ff_rel_ack_b;
    wire        fifo_arm_ack_all = ff_arm_ack_a & ff_arm_ack_b;
    wire        fifo_release_ack_all = ff_rel_ack_a & ff_rel_ack_b;

    // 分时流程独占 arm；软件只负责触发 PWM，不能绕过状态机提前开帧。
    wire fifo_arm_src = fsm_fifo_arm_req;


    always @(posedge clk_samp) begin
        if (rst_samp) begin
            for (bi = 0; bi < 16; bi = bi + 1) bdly[bi] <= 16'h0000;
        end else if (samp_valid) begin
            bdly[0] <= samp_a;
            for (bi = 1; bi < 16; bi = bi + 1) bdly[bi] <= bdly[bi-1];
        end
    end
    wire [15:0] samp_b_eff = (dbg_bdelay == 4'd0) ? samp_b
                                                : bdly[dbg_bdelay - 4'd1];


    // 同发同收：两路每个采样点都写各自的 FIFO
    wire ff_wr_en_a = samp_valid & fsm_acq_en;
    wire ff_wr_en_b = samp_valid & fsm_acq_en;

    Frame_fifo #(.NW(NW), .ADDR_W(ADDR_W), .DW(16)) u_frame_fifo_a (
        .clk_w       (clk_samp),
        .rst_w       (rst_samp),
        .arm_req     (fifo_arm_src),
        .arm_ack     (ff_arm_ack_a),
        .release_req (fsm_fifo_release_req),
        .release_ack (ff_rel_ack_a),
        .wr_en       (ff_wr_en_a),
        .samp        (samp_a),                          // 仅 A 轮写入
        .clk_r       (clk_sys),
        .rst_r       (rst_sys),
        .rd_addr     (fifo_rd_addr_a[ADDR_W-1:0]),
        .rd_data     (ff_rdata_a),
        .frame_ready (ff_ready_a),
        .busy        (ff_busy_a),
        .wcount      (ff_wcount_a)
    );

    Frame_fifo #(.NW(NW), .ADDR_W(ADDR_W), .DW(16)) u_frame_fifo_b (
        .clk_w       (clk_samp),
        .rst_w       (rst_samp),
        .arm_req     (fifo_arm_src),
        .arm_ack     (ff_arm_ack_b),
        .release_req (fsm_fifo_release_req),
        .release_ack (ff_rel_ack_b),
        .wr_en       (ff_wr_en_b),
        .samp        (samp_b_eff),                      // 仅 B 轮写入
        .clk_r       (clk_sys),
        .rst_r       (rst_sys),
        .rd_addr     (fifo_rd_addr_b[ADDR_W-1:0]),
        .rd_data     (ff_rdata_b),
        .frame_ready (ff_ready_b),
        .busy        (ff_busy_b),
        .wcount      (ff_wcount_b)
    );

    // 两路都写满才算一帧就绪
    assign frame_ready = ff_ready_a & ff_ready_b;

    wire [15:0] ff_wcount_cur = (ff_wcount_a < ff_wcount_b) ? ff_wcount_a : ff_wcount_b;
    wire        ff_ready_cur  = ff_ready_a & ff_ready_b;
    // NW 是地址位宽，不能拿它当样本数；N 才是 256。
    wire        fifo_full_cur = ff_ready_cur |
                                (ff_wcount_cur >= N[15:0]);

    wire da_ack_all = da_ack_0 & da_ack_1;


    wire uart_tx_full, uart_tx_busy;
    wire scr_tx_push;
    wire [7:0] scr_tx_byte;
    wire scr_busy;
    wire scr_flush_done;
    wire [15:0] scr_pt_count;


    wire        cpu_uart_push = uart_push & ~uart_tx_full;
    wire        scr_uart_push = scr_tx_push & ~uart_push & ~uart_tx_full;
    wire        u_push = cpu_uart_push | scr_uart_push;
    wire [7:0]  u_data = cpu_uart_push ? uart_byte : scr_tx_byte;


    uart_tx #(.FIFO_AW(10), .DIV_W(16)) u_uart (   /* 1024 字节够用：1s 刷 800 点仅占 29% 带宽，实测 +EBR 2 块不划算 */
        .clk     (clk_sys),
        .rst     (rst_sys),
        .tx_push (u_push),
        .tx_byte (u_data),
        .tx_flush(scr_abort),
        .tx_full (uart_tx_full),
        .tx_busy (uart_tx_busy),
        .div_cfg (uart_div),
        .div_we  (uart_div_we),
        .txd     (screen_txd)
    );


    wire scr_boot_done;
    tjc_screen #(.PT_AW(11)) u_tjc (
        .clk         (clk_sys),
        .rst         (rst_sys),
        .flush_go    (scr_flush),
        .cle_go      (scr_cle),
        .purge_go    (scr_purge),
        .abort       (scr_abort),
        .busy        (scr_busy),
        .flush_done  (scr_flush_done),
        .boot_done   (scr_boot_done),
        .pt_we       (scr_pt_we),
        .pt_data     (scr_pt_data),
        .pt_clr      (scr_pt_clr),
        .pt_count    (scr_pt_count),
        .fld_we      (scr_fld_we),
        .fld_id      (scr_fld_id),
        .fld_type    (scr_fld_type),
        .fld_sign    (scr_fld_sign),
        .fld_val     (scr_fld_val),
        .curve_ch    (scr_curve_ch),
        .curve_obj   (scr_curve_obj),
        .tx_push     (scr_tx_push),
        .tx_byte     (scr_tx_byte),
        .tx_full     (uart_tx_full),
        .tx_line_busy(uart_tx_busy),
        .rx_boot     (1'b0),
        .rx_ready    (t_fe),
        .rx_done     (t_fd)
    );


    PWM_gen #(.PER_W(16)) u_pwm (
        .clk          (clk_sys),
        .rst_n        (~rst_sys),          // State_machine 族统一用 rst_n
        .start_pwm    (start_pwm_w),
        .freq_pwm     (freq_pwm[15:0]),
        .pwm_cycles   (pwm_cycles),
        .pwm_out1     (pwm_out1),
        .pwm_out2     (pwm_out2),
        .pwm_ack      (pwm_ack_w),
        .burst_active (pwm_burst_active)
    );


    Delay_gen #(.CNT_W(32)) u_delay1 (
        .clk         (clk_sys),
        .rst_n       (~rst_sys),
        .start_delay (start_delay1_w),
        .delay_cnt   (delay_1),
        .delay_ack   (delay1_ack_w)
    );


    // DA 在采集窗口开启的同一拍写入增益码（窗口内恒定，不需要逐点刷新）
    DAC7811_Ctrl #(.CLK_DIV(2), .DAC_CODE(12'd2048)) u_dac0 (
        .clk        (clk_sys),
        .rst_n      (~rst_sys),
        .start_ADDA (fsm_acq_en),
        .da_code    (da_code),
        .sync_n     (da_sync_0_o),
        .sclk       (da_sclk_0_o),
        .sdin       (da_sdin_0_o),
        .da_ack     (da_ack_0)
    );


    assign da_sync_1_o = da_sync_0_o;
    assign da_sclk_1_o = da_sclk_0_o;
    assign da_sdin_1_o = da_sdin_0_o;
    assign da_ack_1    = da_ack_0;


    State_machine #(.GAP_CYC(32'd100000)) u_fsm (      // 轮间 1 ms @100 MHz
        .clk              (clk_sys),
        .rst_n            (~rst_sys),
        .start            (start_pwm),          // APB 写 PWM_CTRL bit0 的上升沿
        .timeout          (timeout_cfg),
        .start_pwm        (start_pwm_w),   .pwm_ack      (pwm_ack_w),
        .start_delay1     (start_delay1_w),.delay1_ack   (delay1_ack_w),
        .fifo_full        (fifo_full_cur),
        .read_all_done    (rd_a_seen & rd_b_seen),
        .fifo_arm_ack     (fifo_arm_ack_all),
        .fifo_release_ack (fifo_release_ack_all),
        .fifo_arm         (fsm_fifo_arm_req),
        .fifo_release     (fsm_fifo_release_req),
        .acq_en           (fsm_acq_en),
        .busy             (meas_busy),
        .done             (fsm_done),
        .error            (fsm_error),
        .error_code       (fsm_error_code8),
        .state_dbg        (fsm_state_3)
    );


    // 2b. 屏回传事件锁存（读 SCR_RX_STAT 自动清）

    always @(posedge clk_sys) begin
        if (rst_sys) begin
            rx_st_a5 <= 1'b0;
            rx_last  <= 8'd0;
        end else begin
            if (scr_rx_en && t_a5) begin
                rx_st_a5 <= 1'b1;
                rx_last  <= t_last;          /* 命令字节锁住，空格不覆盖 */
            end
            if (apb_rd && (PADDR[7:2] == 6'h2D)) rx_st_a5 <= 1'b0;
        end
    end


    // 3. APB 读多路

    reg [31:0] prdata_mux;

    always @(*) begin
        case (PADDR[7:2])
            6'h00: prdata_mux = {31'd0, fifo_win_gate};          // FIFO_CTRL 回读


            6'h01: prdata_mux = {8'd0, ff_wcount_a[7:0], 12'd0,
                                 ff_ready_b, ff_busy_b, ff_ready_a, ff_busy_a};
            6'h02: prdata_mux = {16'd0, ff_rdata_a};             // FIFO_DATA_A
            6'h05: prdata_mux = {16'd0, ff_rdata_b};             // FIFO_DATA_B
            6'h03: prdata_mux = {ff_wcount_b, ff_wcount_a};      // FIFO_CNT {B,A}
            6'h04: prdata_mux = 32'h46524651;                    // "FRFQ"
            6'h10: prdata_mux = {31'd0, scr_busy};               // SCR_CTRL 回读
            6'h11: prdata_mux = {29'd0, scr_boot_done, uart_tx_busy,
                                 scr_busy};              // SCR_STAT
            6'h14: prdata_mux = fld_snapshot;                // SCR_FIELD 回读
            6'h15: prdata_mux = {16'd0, scr_pt_count};       // SCR_PTMAX 回读
            6'h21: prdata_mux = {30'd0, uart_tx_busy, uart_tx_full}; // UART_STAT


            6'h2A: prdata_mux = {28'd0, dbg_bdelay};         // DBG_DELAY 回读
            6'h2C: prdata_mux = {15'd0, scr_rx_en, scr_rx_div};  // SCR_RX_CTRL 回读
            6'h2D: prdata_mux = {23'd0, rx_st_a5, 8'd0};  // bit8，软核读 0x100
            6'h2E: prdata_mux = {24'd0, rx_last};    // 锁住的命令字节


            // bit2=win bit3=done bit4=err bit5=接收通道(0=A 1=B)
            // 软件靠 bit5 精确判断本帧来自哪个接收通道，不用靠"哪路 ready"反推。
            6'h28: prdata_mux = {27'd0, fsm_error, fsm_done,
                                 fsm_acq_en, meas_busy, pwm_burst_active};


            // 8+8+4+4 = 24 位，前面补 8 位 0 → 32 位


            6'h09: prdata_mux = {8'd0, fsm_error_code8, dsp_done_cnt[7:0],
                                 4'd0, fsm_done, meas_busy, fsm_error, 1'b0};

            6'h25: prdata_mux = freq_pwm;           // PWM_PERIOD(0x94)
            6'h26: prdata_mux = {16'd0, pwm_cycles};// PWM_CYCLES(0x98)
            6'h27: prdata_mux = delay_1;            // PWM_DELAY(0x9C)=Delay_1

            6'h06: prdata_mux = 32'd0;
            6'h0C: prdata_mux = timeout_cfg;   // TIMEOUT_ALL 回读
            6'h0D: prdata_mux = timeout_cfg;
            6'h0E: prdata_mux = timeout_cfg;
            6'h0F: prdata_mux = timeout_cfg;
            // DA_CODE(0x1C) 回读
            6'h07: prdata_mux = {20'd0, da_code};
            // MEAS_STATE(0x28) 回读：one-hot 状态向量（32 位完整）
            6'h0A: prdata_mux = {29'd0, fsm_state_3};   // 3 位状态码
            default: prdata_mux = 32'hDEADBEEF;
        endcase
    end

    assign PRDATA = prdata_mux;

    // 采集窗口使能：状态机只在 ACQ 态给出，用于门控 ADC 与 FIFO 写入
    assign acq_en_o = fsm_acq_en;

endmodule

`default_nettype wire