`timescale 1ns/1ps


`default_nettype none

module Frame_fifo #(
    parameter integer NW    = 9,          // 每路样本数 = 2^NW = 512
    parameter integer ADDR_W = 9,         // RAM 地址位宽（512 深）
    parameter integer DW    = 16          // 数据位宽：16 bit，承载 12 位 ADC 码
) (

    input  wire                 clk_w,
    input  wire                 rst_w,
    input  wire                 arm_req,   // 单拍：请求开新帧（读域）
    output wire                 arm_ack,   // 单拍：新帧已开且旧 ready 已撤（读域）
    input  wire                 release_req, // 单拍：请求交还本帧所有权（读域）
    output wire                 release_ack, // 单拍：frame_ready 已撤（读域）
    input  wire                 wr_en,     // 样本有效（单周期脉冲）
    input  wire [DW-1:0]        samp,


    input  wire                 clk_r,
    input  wire                 rst_r,
    input  wire [ADDR_W-1:0]    rd_addr,
    output wire [DW-1:0]        rd_data,
    output wire                 frame_ready, // 一帧已收满（同步到读域）
    output wire                 busy,        // 正在收（同步到读域）
    output wire [15:0]          wcount       // 本帧已收样本数（同步到读域）
);

    localparam integer N = (1 << NW);

    (* ram_style = "block" *) reg [DW-1:0] mem [0:N-1];


    // arm：读域脉冲 → 翻转 → 写域边沿 → 开帧 → ack 脉冲同步回读域

    reg arm_t;                 // 读域翻转寄存器（CDC 安全载体）
    reg arm_t_w1, arm_t_w2, arm_t_w3;
    reg arm_open_w;            // 写域：本拍开新帧
    reg arm_ack_w1, arm_ack_w2;

    reg rel_t;                 // release 的翻转载体（同样 CDC 安全）
    reg rel_t_w1, rel_t_w2, rel_t_w3;
    reg rel_do_w;              // 写域：本拍执行释放
    reg rel_ack_w1, rel_ack_w2;

    always @(posedge clk_r) begin
        if (rst_r) arm_t <= 1'b0;
        else       arm_t <= arm_t ^ arm_req;
    end

    always @(posedge clk_w) begin
        if (rst_w) begin
            arm_t_w1 <= 1'b0; arm_t_w2 <= 1'b0; arm_t_w3 <= 1'b0;
        end else begin
            arm_t_w1 <= arm_t;
            arm_t_w2 <= arm_t_w1;
            arm_t_w3 <= arm_t_w2;
        end
    end

    wire arm_edge_w = (arm_t_w2 ^ arm_t_w3);   // 写域看到的翻转沿

    // ack 回读域（写域脉冲 → 两级同步）
    always @(posedge clk_w) begin
        if (rst_w) begin
            arm_ack_w1 <= 1'b0; arm_ack_w2 <= 1'b0;
        end else begin
            arm_ack_w1 <= arm_open_w;
            arm_ack_w2 <= arm_ack_w1;
        end
    end

    assign arm_ack = arm_ack_w2;


    always @(posedge clk_r) begin
        if (rst_r) rel_t <= 1'b0;
        else       rel_t <= rel_t ^ release_req;
    end

    always @(posedge clk_w) begin
        if (rst_w) begin
            rel_t_w1 <= 1'b0; rel_t_w2 <= 1'b0; rel_t_w3 <= 1'b0;
        end else begin
            rel_t_w1 <= rel_t;
            rel_t_w2 <= rel_t_w1;
            rel_t_w3 <= rel_t_w2;
        end
    end

    wire rel_edge_w = (rel_t_w2 ^ rel_t_w3);

    always @(posedge clk_w) begin
        if (rst_w) begin
            rel_ack_w1 <= 1'b0; rel_ack_w2 <= 1'b0;
        end else begin
            rel_ack_w1 <= rel_do_w;
            rel_ack_w2 <= rel_ack_w1;
        end
    end

    assign release_ack = rel_ack_w2;


    // 写侧

    reg [ADDR_W-1:0] wptr;
    reg              full_w;
    reg [15:0]       wcount_w;

    always @(posedge clk_w) begin
        if (rst_w) begin
            wptr       <= {ADDR_W{1'b0}};
            full_w     <= 1'b0;
            wcount_w   <= 16'd0;
            arm_open_w <= 1'b0;
        end else begin
            arm_open_w <= arm_edge_w;      // 单拍：本帧已开
            rel_do_w   <= rel_edge_w;      // 单拍：本帧已释放
            if (rel_edge_w) begin
                // 交还所有权：撤掉 frame_ready，允许下一轮 arm 重开。


                full_w <= 1'b0;
            end else if (arm_edge_w) begin
                // 重新开帧：指针归零，从这一刻起才算样本；同时撤掉旧 ready
                wptr     <= {ADDR_W{1'b0}};
                full_w   <= 1'b0;
                wcount_w <= 16'd0;
            end else if (wr_en && !full_w) begin
                mem[wptr]  <= samp;
                wcount_w   <= wcount_w + 16'd1;
                if (wptr == N[ADDR_W-1:0] - 1'b1)
                    full_w <= 1'b1;
                else
                    wptr <= wptr + 1'b1;
            end
        end
    end


    // 读侧：异步读

    assign rd_data = mem[rd_addr];


    // 状态同步（写域 → 读域）


    reg full_r1, full_r2;
    reg [15:0] cnt_r1, cnt_r2;
    always @(posedge clk_r) begin
        if (rst_r) begin
            full_r1 <= 1'b0; full_r2 <= 1'b0;
            cnt_r1  <= 16'd0; cnt_r2  <= 16'd0;
        end else begin
            full_r1 <= full_w;   full_r2 <= full_r1;
            cnt_r1  <= wcount_w; cnt_r2  <= cnt_r1;
        end
    end

    assign frame_ready = full_r2;
    assign wcount      = cnt_r2;
    // "busy" = 已收到样本但还没满（收满瞬间转 false，表示该读了）
    assign busy        = (cnt_r2 != 16'd0) && !full_r2;

endmodule

`default_nettype wire
