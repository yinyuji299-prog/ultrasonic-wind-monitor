`timescale 1ns/1ps


`default_nettype none

module dec_fmt (
    input  wire         clk,
    input  wire         rst,

    input  wire         start,        // 单周期脉冲，启动一次转换
    input  wire [23:0]  value,        // 0 .. 99,999,999

    output reg          busy,
    output reg          done,         // 单周期脉冲，转换完成
    output reg  [3:0]   ndig,         // 有效位数 1..8
    output reg  [31:0]  digits_flat   // 8 位 BCD，[3:0]=个位
);

    // 10^k 表，k=0..7
    function [23:0] pow10;
        input [3:0] k;
        begin
            case (k)
                4'd0: pow10 = 24'd1;
                4'd1: pow10 = 24'd10;
                4'd2: pow10 = 24'd100;
                4'd3: pow10 = 24'd1000;
                4'd4: pow10 = 24'd10000;
                4'd5: pow10 = 24'd100000;
                4'd6: pow10 = 24'd1000000;
                4'd7: pow10 = 24'd10000000;
                default: pow10 = 24'd0;
            endcase
        end
    endfunction

    localparam S_IDLE = 2'd0;
    localparam S_SUB  = 2'd1;
    localparam S_NEXT = 2'd2;
    localparam S_FIN  = 2'd3;

    reg [1:0]  st;
    reg [23:0] rem;
    reg [3:0]  k;
    reg [3:0]  dcur;
    reg [31:0] acc;


    function [3:0] lead_ndig;
        input [31:0] a;
        integer i;
        reg [3:0] r;
        begin
            r = 4'd1;
            for (i = 1; i <= 7; i = i + 1) begin
                if (a[i*4 +: 4] != 4'd0)
                    r = i[3:0] + 4'd1;
            end
            lead_ndig = r;
        end
    endfunction

    always @(posedge clk) begin
        if (rst) begin
            st          <= S_IDLE;
            busy        <= 1'b0;
            done        <= 1'b0;
            ndig        <= 4'd1;
            digits_flat <= 32'd0;
            rem         <= 24'd0;
            k           <= 4'd7;
            dcur        <= 4'd0;
            acc         <= 32'd0;
        end else begin
            done <= 1'b0;
            case (st)
                S_IDLE: begin
                    busy <= 1'b0;
                    if (start) begin
                        rem  <= value;
                        k    <= 4'd7;
                        dcur <= 4'd0;
                        acc  <= 32'd0;
                        busy <= 1'b1;
                        st   <= S_SUB;
                    end
                end

                S_SUB: begin
                    if (rem >= pow10(k)) begin
                        rem  <= rem - pow10(k);
                        dcur <= dcur + 4'd1;   // 最多到 9（value < 10^8 保证）
                    end else
                        st <= S_NEXT;
                end

                S_NEXT: begin
                    acc[k*4 +: 4] <= dcur;
                    dcur          <= 4'd0;
                    if (k == 4'd0) begin

                        digits_flat <= {acc[31:4], dcur};
                        ndig        <= lead_ndig({acc[31:4], dcur});
                        busy        <= 1'b0;
                        done        <= 1'b1;
                        st          <= S_IDLE;
                    end else begin
                        k  <= k - 4'd1;
                        st <= S_SUB;
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule

`default_nettype wire