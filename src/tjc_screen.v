`timescale 1ns/1ps


`default_nettype none

module tjc_screen #(
    parameter integer PT_AW = 11             // 点缓冲深度 = 2048。10 位地址写到 512 绕回，800 点只剩 288 列
) (
    input  wire             clk,
    input  wire             rst,


    input  wire             flush_go,        // 整帧透传到曲线控件
    input  wire             cle_go,          // 清曲线
    input  wire             purge_go,        // 手动 00 FF FF FF
    input  wire             abort,           // 新按键：停掉正在发的波形
    output reg              busy,            // 正在发送
    output reg              flush_done,      // 整帧刷完（单周期脉冲）
    output wire             boot_done,       // 上电延时已过


    input  wire             pt_we,
    input  wire [7:0]       pt_data,
    input  wire             pt_clr,
    output reg  [15:0]      pt_count,


    input  wire             fld_we,
    input  wire [7:0]       fld_id,          // 控件编号 0..31
    input  wire [1:0]       fld_type,        // 00=t.txt 01=n.val 10=x.val
    input  wire             fld_sign,        // n/x 时前置 '-'
    input  wire [23:0]      fld_val,         // 无符号幅值


    input  wire [1:0]       curve_ch,
    input  wire [1:0]       curve_obj,      /* 2026-11-01 曲线控件号 0/1/2 (s0/s1/s2) */


    output reg              tx_push,
    output reg  [7:0]       tx_byte,
    input  wire             tx_full,
    input  wire             tx_line_busy,    // UART 移位器忙（尾排空判定）


    // 0x88 = 系统启动成功 -> 取代盲等 1.5s
    // 0xFE = 数据透传就绪 -> 取代盲等 50ms
    // 0xFD = 透传完成    -> 提前结束排空等待
    input  wire             rx_boot,
    input  wire             rx_ready,
    input  wire             rx_done
);

    localparam [15:0] PTS16   = 16'd256;             // 2026-11-01 三条曲线各 256 点（原始波形点数），带宽 3KB/帧
    localparam [23:0] ADDTW24 = 24'd10_000_000;      // 官方 C 示例 delay_ms(100) 后直接发点
    localparam [27:0] BOOTW28 = 28'd150_000_000;     // 上电等屏 1.5s@100MHz
    localparam [31:0] FLUSH_MIN = 32'd200_000;       // 两帧最小间隔 2ms


    // 1. 曲线点缓冲

    (* ram_style = "block" *) reg [7:0] ptbuf [0:(1<<PT_AW)-1];

    reg  [PT_AW-1:0] pt_wp;
    reg  [PT_AW-1:0] rd_pt_addr;   // 只由 FSM 原始流阶段驱动（单一驱动者）


    wire pt_writable = (pt_count < PTS16);

    always @(posedge clk) begin
        if (rst) begin
            pt_wp    <= {PT_AW{1'b0}};
            pt_count <= 16'd0;
        end else if (pt_clr) begin
            pt_wp    <= {PT_AW{1'b0}};
            pt_count <= 16'd0;
        end else if (pt_we && pt_writable) begin
            ptbuf[pt_wp] <= pt_data;
            pt_wp        <= pt_wp + 1'b1;
            pt_count     <= pt_count + 16'd1;
        end
    end


    reg [7:0] pt_rd;
    always @(posedge clk) begin
        pt_rd <= ptbuf[rd_pt_addr];
    end


    // 2. 十进制格式化器（数值域；累减、无前导零、≤72 拍）

    reg         df_start;
    wire        df_busy, df_done;
    wire [3:0]  df_ndig;
    wire [31:0] df_digits;
    reg         df_rdy;                               

    reg  [7:0]  f_id_r;                                    
    reg  [1:0]  f_type_r;
    reg         f_sign_r;
    reg  [23:0] fld_val_r;              // 同时是 dec_fmt 的输入（转换期间稳定）

    dec_fmt u_dec (
        .clk         (clk),
        .rst         (rst),
        .start       (df_start),
        .value       (fld_val_r),
        .busy        (df_busy),
        .done        (df_done),
        .ndig        (df_ndig),
        .digits_flat (df_digits)
    );


    // 3. qty 数字化器（点数 1..800 → 十进制 1..3 位，多拍累减，每帧一次）

    localparam Q_SET = 2'd0, Q_H = 2'd1, Q_T = 2'd2, Q_RDY = 2'd3;
    reg [1:0]  q_st;
    reg [15:0] q_rem;
    reg [3:0]  q_100, q_10, q_1;
    reg        q_ok;
    wire [1:0] q_ndig = (q_100 != 4'd0) ? 2'd3
                       : (q_10  != 4'd0) ? 2'd2 : 2'd1;


    // 4. ASCII 常量

    localparam [7:0] A_a=8'h61, A_d=8'h64, A_t=8'h74, A_x=8'h78, A_s=8'h73,
                     A_i=8'h69, A_dot=8'h2E, A_com=8'h2C, A_sp=8'h20,
                     A_eq=8'h3D, A_quo=8'h22, A_n=8'h6E, A_v=8'h76,
                     A_l=8'h6C, A_c=8'h63, A_e=8'h65, A_m=8'h2D,
                     A_0=8'h30, A_1=8'h31, A_2=8'h32, A_5=8'h35, FF=8'hFF;

    function [7:0] ascii_of;
        input [3:0] d;
        begin
            ascii_of = 8'd48 + {4'd0, d};
        end
    endfunction


    wire id_2ch = (f_id_r >= 8'd10);
    reg  [7:0] id_hi, id_lo;
    always @(*) begin
        if (f_id_r >= 8'd20) begin
            if (f_id_r >= 8'd30) begin id_hi = 8'd3; id_lo = f_id_r - 8'd30; end
            else                 begin id_hi = 8'd2; id_lo = f_id_r - 8'd20; end
        end else begin
            if (f_id_r >= 8'd10) begin id_hi = 8'd1; id_lo = f_id_r - 8'd10; end
            else                 begin id_hi = 8'd0; id_lo = f_id_r;        end
        end
    end

    wire       w_txt   = (f_type_r == 2'b00);
    wire [7:0] w_idn   = id_2ch ? 8'd2 : 8'd1;          // 控件名数字位数
    wire [7:0] w_alen  = w_txt ? 8'd5 : 8'd4;           // "txt=\"" : "val="
    wire [7:0] w_pdot  = 8'd1 + w_idn;                  // '.' 的位置
    wire [7:0] w_patr  = w_pdot + 8'd1;                 // 属性串起点
    wire       w_sign  = (!w_txt) && f_sign_r;          // '-' 仅 n/x
    wire [7:0] w_psign = w_patr + w_alen;               // '-' 的位置
    wire [7:0] w_pdig  = w_psign + (w_sign ? 8'd1 : 8'd0);   // 数字段起点
    wire [7:0] w_pff   = w_pdig + {4'd0, df_ndig} + (w_txt ? 8'd1 : 8'd0);
    wire [7:0] w_last  = w_pff + 8'd2;                  // 最后一片 FF

    reg in_dig;


    reg [7:0]  seq;
    always @(*) in_dig = (seq >= w_pdig) && (seq < (w_pdig + {4'd0, df_ndig}));

    reg  [7:0] f_k;
    reg  [3:0] f_kofs;
    reg  [3:0] f_dig;
    reg  [5:0] f_bpos;
    always @(*) begin
        f_k    = seq - w_pdig;
        f_kofs = df_ndig - 4'd1 - f_k[3:0];
        f_bpos = {f_kofs, 2'b00};
        f_dig  = df_digits[f_bpos +: 4];
    end


    wire emit_ok_f = (seq >= w_pdig) ? df_rdy : 1'b1;


    // 6. FLUSH 指令头分段


    wire [7:0] q_last = 8'd15 + {6'd0, q_ndig};

    reg [7:0] q_k;
    reg [1:0] q_kofs;
    reg [3:0] q_nib;
    always @(*) begin
        q_k    = seq - 8'd13;
        q_kofs = q_ndig - 2'd1 - q_k[1:0];
        case (q_kofs)
            2'd0:    q_nib = q_1;
            2'd1:    q_nib = q_10;
            2'd2:    q_nib = q_100;
            default: q_nib = 4'd0;
        endcase
    end

    reg in_qdig;
    always @(*) in_qdig = (seq >= 8'd13) && (seq <= (8'd12 + {6'd0, q_ndig}));

    // 数字段和它后面的 FF 数量依赖 q_ndig → seq>=13 起统一等 q_ok
    wire emit_ok_h = (seq >= 8'd13) ? q_ok : 1'b1;


    // 7. 上电时序与刷屏节流

    reg [27:0] boot_cnt;
    reg        boot_done_r;
    reg        boot_pulse;                        // boot_done 上升沿单拍
    assign boot_done = boot_done_r;

    always @(posedge clk) begin
        boot_pulse <= 1'b0;
        if (rst) begin
            boot_cnt    <= BOOTW28;
            boot_done_r <= 1'b0;
        end else if (!boot_done_r && rx_boot) begin


            boot_done_r <= 1'b1;
            boot_pulse  <= 1'b1;                  // 通知 FSM 插队清洗一次
        end else if (boot_cnt != 28'd0) begin
            boot_cnt <= boot_cnt - 28'd1;                             
        end else if (!boot_done_r) begin
            boot_done_r <= 1'b1;
            boot_pulse  <= 1'b1;
        end
    end

    reg  [31:0] flush_gap;
    reg         flush_arm;                          // 重装脉冲（单一驱动者）
    always @(posedge clk) begin
        if (rst)                     flush_gap <= FLUSH_MIN;
        else if (flush_arm)          flush_gap <= FLUSH_MIN;
        else if (flush_gap != 32'd0) flush_gap <= flush_gap - 32'd1;
    end
    wire flush_ok = (flush_gap == 32'd0) && boot_done_r;


    localparam T_NONE  = 3'd0;
    localparam T_FLUSH = 3'd1;
    localparam T_FIELD = 3'd2;
    localparam T_CLE   = 3'd3;
    localparam T_PURGE = 3'd4;

    reg [2:0]  cur_job;
    reg [2:0]  fl_ph;                 // FLUSH 阶段：0头 1等待 2原始流 3尾 4排空
    reg [23:0] addt_cnt;
    reg [15:0] qty_r;
    reg [15:0] raw_sent;
    reg [7:0]  byte_val;

    reg        purge_pend;            // 由本 FSM 独占（boot_pulse / purge_go 置位）
    reg        fld_pend;
    reg        flush_pend;
    /* 2026-11-01 请求时刻锁存控件号/通道号：组帧时实时读寄存器会被软件改掉，
     * 导致某一帧带着下一帧的控件号发出（s1 被 s2 顶掉）。 */
    reg [1:0]  flush_obj_l, flush_ch_l;
    reg [1:0]  cle_obj_l;
    reg        cle_pend;

    always @(posedge clk) begin
        if (rst) begin
            cur_job    <= T_NONE;
            seq        <= 8'd0;
            tx_push    <= 1'b0;
            tx_byte    <= 8'h00;
            busy       <= 1'b0;
            flush_done <= 1'b0;
            df_start   <= 1'b0;
            df_rdy     <= 1'b0;
            fl_ph      <= 3'd0;
            addt_cnt   <= 24'd0;
            qty_r      <= 16'd0;
            raw_sent   <= 16'd0;
            rd_pt_addr <= {PT_AW{1'b0}};
            fld_pend   <= 1'b0;
            flush_pend <= 1'b0;
            cle_pend   <= 1'b0;
            purge_pend <= 1'b0;
            flush_arm  <= 1'b0;
            f_id_r     <= 8'd0;
            f_type_r   <= 2'b00;
            f_sign_r   <= 1'b0;
            fld_val_r  <= 24'd0;
            q_st       <= Q_SET;
            q_rem      <= 10'd0;
            q_100      <= 4'd0;
            q_10       <= 4'd0;
            q_1        <= 4'd0;
            q_ok       <= 1'b0;
        end else begin

            tx_push    <= 1'b0;
            flush_done <= 1'b0;
            df_start   <= 1'b0;
            flush_arm  <= 1'b0;


            if (fld_we) begin
                f_id_r    <= fld_id;
                f_type_r  <= fld_type;
                f_sign_r  <= fld_sign;
                fld_val_r <= fld_val;
                fld_pend  <= 1'b1;
            end
            if (flush_go) flush_pend <= 1'b1;
            if (cle_go)   cle_pend   <= 1'b1;
            if (purge_go || boot_pulse) purge_pend <= 1'b1;   // 手动 / 上电自动清洗


            if (df_start) df_rdy <= 1'b0;          // 派发拍清零
            if (df_done)  df_rdy <= 1'b1;          // done 不会与 start 同拍


            case (q_st)
            Q_SET: begin
                q_ok  <= 1'b0;
                q_100 <= 4'd0;
                q_10  <= 4'd0;
                q_rem <= qty_r;
                q_st  <= Q_H;
            end
            Q_H: begin
                if (q_rem >= 16'd100) begin q_rem <= q_rem - 16'd100; q_100 <= q_100 + 4'd1; end
                else q_st <= Q_T;
            end
            Q_T: begin
                if (q_rem >= 16'd10) begin q_rem <= q_rem - 16'd10; q_10 <= q_10 + 4'd1; end
                else begin
                    q_1  <= q_rem[3:0];
                    q_ok <= 1'b1;
                    q_st <= Q_RDY;
                end
            end
            default: ;                              // Q_RDY 停等；下一帧由派发端重置到 Q_SET
            endcase


            if (abort) begin
                cur_job    <= T_NONE;
                busy       <= 1'b0;
                flush_pend <= 1'b0;
                cle_pend   <= 1'b0;
                fld_pend   <= 1'b0;
            end


            if (cur_job == T_NONE && !abort) begin
                busy <= 1'b0;
                if (purge_pend && boot_done_r) begin
                    purge_pend <= 1'b0;
                    cur_job    <= T_PURGE;
                    seq        <= 8'd0;
                    busy       <= 1'b1;
                end else if (boot_done_r && fld_pend) begin
                    fld_pend <= 1'b0;
                    cur_job  <= T_FIELD;
                    seq      <= 8'd0;
                    df_start <= 1'b1;
                    busy     <= 1'b1;
                end else if (boot_done_r && cle_pend) begin
                    cle_pend   <= 1'b0;
                    cle_obj_l  <= curve_obj;       /* ★ 锁存控件号 */
                    cur_job  <= T_CLE;
                    seq      <= 8'd0;
                    busy     <= 1'b1;
                end else if (flush_pend && (pt_count != 16'd0) && flush_ok) begin
                    flush_pend <= 1'b0;
                    flush_obj_l <= curve_obj;      /* ★ 锁存控件号 */
                    flush_ch_l  <= curve_ch;       /* ★ 锁存通道号 */
                    cur_job    <= T_FLUSH;
                    seq        <= 8'd0;
                    fl_ph      <= 3'd0;
                    qty_r      <= PTS16;          /* 命令和发送都是 800，不被计数截成 512 */
                    raw_sent   <= 16'd0;
                    rd_pt_addr <= {PT_AW{1'b0}};
                    flush_arm  <= 1'b1;
                    q_st       <= Q_SET;
                    busy       <= 1'b1;
                end
            end


            case (cur_job)

            T_PURGE: begin                       // 00 FF FF FF
                byte_val = (seq == 8'd0) ? 8'h00 : FF;
                if (!tx_full) begin
                    tx_push <= 1'b1;
                    tx_byte <= byte_val;
                    if (seq == 8'd3) begin
                        cur_job <= T_NONE;
                        busy    <= 1'b0;
                    end else
                        seq <= seq + 8'd1;
                end
            end


            T_CLE: begin                         // "cle s<N>,0" FF FF FF（11 字节）
                /* 2026-11-01 原实现发 "cle <数字>,0"，那个数字是【对象 id】；
                 * 而屏工程里 s0=1、s1=7、s2=10，所以 cle 0/1/2 清的是
                 * page0 / s0 / n0 —— s1、s2 从来没被清过，还误清了 n0。
                 * 改成与 addt 一致的控件名形式："cle s<N>,0"。 */
                case (seq)
                    8'd0:  byte_val = A_c;
                    8'd1:  byte_val = A_l;
                    8'd2:  byte_val = A_e;
                    8'd3:  byte_val = A_sp;
                    8'd4:  byte_val = A_s;                       /* 's' */
                    8'd5:  byte_val = ascii_of({2'b0, cle_obj_l});  /* 锁存值 */
                    8'd6:  byte_val = A_com;
                    8'd7:  byte_val = A_0;
                    default: byte_val = FF;      // 8/9/10
                endcase
                if (!tx_full) begin
                    tx_push <= 1'b1;
                    tx_byte <= byte_val;
                    if (seq == 8'd10) begin
                        cur_job <= T_NONE;
                        busy    <= 1'b0;
                    end else
                        seq <= seq + 8'd1;
                end
            end


            // "tN.txt=\"<d>\"" / "nN.val=<d>" / "xN.val=<d>" + FF FF FF

            T_FIELD: begin
                if (seq == 8'd0)
                    byte_val = w_txt ? A_t : ((f_type_r == 2'b01) ? A_n : A_x);
                else if (id_2ch && (seq == 8'd1))
                    byte_val = ascii_of(id_hi[3:0]);
                else if (seq == (w_pdot - 8'd1))
                    byte_val = ascii_of(id_lo[3:0]);
                else if (seq == w_pdot)
                    byte_val = A_dot;
                else if ((seq >= w_patr) && (seq < w_psign)) begin
                    case (seq - w_patr)
                        8'd0:    byte_val = w_txt ? A_t : A_v;
                        8'd1:    byte_val = w_txt ? A_x : A_a;
                        8'd2:    byte_val = w_txt ? A_t : A_l;
                        8'd3:    byte_val = A_eq;
                        default: byte_val = A_quo;      // txt=" 的第 5 字符
                    endcase
                end
                else if (w_sign && (seq == w_psign))
                    byte_val = A_m;
                else if (in_dig)
                    byte_val = ascii_of(f_dig);
                else if (w_txt && (seq == (w_pff - 8'd1)))
                    byte_val = A_quo;
                else if (seq >= w_pff)
                    byte_val = FF;
                else
                    byte_val = 8'h00;                   // 理论不可达

                if (!tx_full && emit_ok_f) begin
                    tx_push <= 1'b1;
                    tx_byte <= byte_val;
                    if (seq == w_last) begin
                        cur_job <= T_NONE;
                        busy    <= 1'b0;
                    end else
                        seq <= seq + 8'd1;
                end
            end


            T_FLUSH: begin
                if (fl_ph == 3'd0) begin
                    if (seq <= 8'd12) begin
                        case (seq)
                            8'd0:  byte_val = A_a;
                            8'd1:  byte_val = A_d;
                            8'd2:  byte_val = A_d;
                            8'd3:  byte_val = A_t;
                            8'd4:  byte_val = A_sp;
                            8'd5:  byte_val = A_s;
                            8'd6:  byte_val = ascii_of({2'b0, flush_obj_l});  /* 锁存值 */
                            8'd7:  byte_val = A_dot;
                            8'd8:  byte_val = A_i;
                            8'd9:  byte_val = A_d;
                            8'd10: byte_val = A_com;
                            8'd11: byte_val = ascii_of({2'b0, flush_ch_l});   /* 锁存值 */
                            default: byte_val = A_com;  // 12
                        endcase
                    end else if (in_qdig)
                        byte_val = ascii_of(q_nib);
                    else
                        byte_val = FF;

                    if (!tx_full && emit_ok_h) begin
                        tx_push <= 1'b1;
                        tx_byte <= byte_val;
                        if (seq == q_last) begin
                            fl_ph    <= 3'd1;
                            addt_cnt <= ADDTW24;
                        end else
                            seq <= seq + 8'd1;
                    end
                end
                else if (fl_ph == 3'd1) begin


                    if (rx_ready || (addt_cnt == 24'd0)) begin
                        rd_pt_addr <= {{(PT_AW-1){1'b0}}, 1'b1}; /* 本拍 RAM 读的是地址 0 */
                        fl_ph <= 3'd2;
                    end else begin
                        addt_cnt <= addt_cnt - 24'd1;
                    end
                end
                else if (fl_ph == 3'd2) begin
                    if (raw_sent != qty_r) begin
                        if (!tx_full) begin


                            tx_push    <= 1'b1;
                            tx_byte    <= pt_rd;
                            raw_sent   <= raw_sent + 16'd1;
                            rd_pt_addr <= rd_pt_addr + 1'b1;
                        end
                    end else begin
                        /* 点已经进发送 FIFO。不等 0xFD，800 ms 空等会把按键拖住。 */
                        cur_job    <= T_NONE;
                        busy       <= 1'b0;
                        flush_done <= 1'b1;
                    end
                end
            end


            default: ;
            endcase
        end
    end


endmodule

`default_nettype wire