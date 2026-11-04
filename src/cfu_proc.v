`timescale 1ns/1ps


module cfu_proc (
    input  wire             clk,
    input  wire             reset,

    input  wire             cmd_valid,
    output wire             cmd_ready,
    input  wire [9:0]       cmd_function_id,
    input  wire [31:0]      cmd_inputs_0,
    input  wire [31:0]      cmd_inputs_1,

    output reg              rsp_valid,
    input  wire             rsp_ready,
    output reg  [31:0]      rsp_outputs_0
);


    // helper functions

    function signed [15:0] sxt8;
        input [7:0] x;
        begin
            sxt8 = {{8{x[7]}}, x};
        end
    endfunction

    function [7:0] sat_u8;
        input signed [15:0] x;
        begin
            if (x > 16'sd127)       sat_u8 =  8'd127;
            else if (x < -16'sd128) sat_u8 = -8'd128;
            else                    sat_u8 = x[7:0];
        end
    endfunction

    function [15:0] sat_q15;
        input signed [34:0] x;
        begin
            if      (x >  35'sd32767) sat_q15 = 16'h7FFF;
            else if (x < -35'sd32768) sat_q15 = 16'h8000;
            else                      sat_q15 = x[15:0];
        end
    endfunction

    function [31:0] brev_comb;
        input [31:0] x;
        integer k;
        begin
            for (k = 0; k < 32; k = k + 1)
                brev_comb[k] = x[31-k];
        end
    endfunction

    // 8-input priority encoder (3 LUT levels)
    function [3:0] enc8;
        input [7:0] x;
        begin
            enc8 = x[7] ? 4'd0 : x[6] ? 4'd1 : x[5] ? 4'd2 : x[4] ? 4'd3 :
                   x[3] ? 4'd4 : x[2] ? 4'd5 : x[1] ? 4'd6 : x[0] ? 4'd7 : 4'd8;
        end
    endfunction

    function [5:0] clz_comb;
        input [31:0] x;
        reg [7:0]  bnz, chunk;
        reg [1:0]  top, lzb;
        begin
            bnz   = {|x[31:24], |x[23:16], |x[15:8], |x[7:0]};
            top   = bnz[3] ? 2'd3 : bnz[2] ? 2'd2 : bnz[1] ? 2'd1 : 2'd0;
            chunk = (top == 2'd3) ? x[31:24] :
                    (top == 2'd2) ? x[23:16] :
                    (top == 2'd1) ? x[15:8]  : x[7:0];
            lzb       = 2'd3 - top;
            clz_comb  = (x == 32'd0) ? 6'd32 : ({lzb, 3'b000} + {2'd0, enc8(chunk)});
        end
    endfunction

    function [2:0] pc4;
        input [3:0] x;
        begin
            pc4 = {2'd0,x[0]} + {2'd0,x[1]} + {2'd0,x[2]} + {2'd0,x[3]};
        end
    endfunction

    function [5:0] pc32_comb;
        input [31:0] x;
        reg [5:0] s0, s1, s2, s3, s4, s5, s6, s7;
        begin
            s0 = {3'd0, pc4(x[3:0])};     s1 = {3'd0, pc4(x[7:4])};
            s2 = {3'd0, pc4(x[11:8])};    s3 = {3'd0, pc4(x[15:12])};
            s4 = {3'd0, pc4(x[19:16])};   s5 = {3'd0, pc4(x[23:20])};
            s6 = {3'd0, pc4(x[27:24])};   s7 = {3'd0, pc4(x[31:28])};
            pc32_comb = ((s0+s1)+(s2+s3)) + ((s4+s5)+(s6+s7));
        end
    endfunction


    // stage 0 : command capture

    reg         busy, v1, v2, v3, v4;
    reg  [9:0]  fid_r;
    reg  [31:0] a_r, b_r;


    reg  [31:0] brev_r;
    reg  [5:0]  clz_r, ctz_r, pcnt_r, ham_r;
    reg  [31:0] rnd_sum_r;         // rs1 + 1<<(rs2-1)   for RSFT
    reg  [31:0] dn_rnd_r;          // 1<<(shr-1)         for Q15MLA

    // debug wires for Efinix Logic Analyzer / ILA capture; do not
    // affect synthesis behaviour.
    (* MARK_DEBUG="true" *) wire            dbg_cmd_valid     = cmd_valid;
    (* MARK_DEBUG="true" *) wire            dbg_cmd_ready     = cmd_ready;
    (* MARK_DEBUG="true" *) wire            dbg_rsp_valid     = rsp_valid;
    (* MARK_DEBUG="true" *) wire            dbg_rsp_ready     = rsp_ready;
    (* MARK_DEBUG="true" *) wire            dbg_busy          = busy;
    (* MARK_DEBUG="true" *) wire            dbg_v1            = v1;
    (* MARK_DEBUG="true" *) wire            dbg_v2            = v2;
    (* MARK_DEBUG="true" *) wire            dbg_v3            = v3;
    (* MARK_DEBUG="true" *) wire            dbg_v4            = v4;
    (* MARK_DEBUG="true" *) wire [9:0]      dbg_fid_r         = fid_r;


    reg  signed [31:0] sat_add_r;  // saturating rs1 + rs2
    reg  signed [31:0] sat_sub_r;  // saturating rs1 - rs2
    reg  signed [31:0] bmag_r;     //  |rs2|
    reg  signed [31:0] nbmag_r;    // -|rs2|
    reg         [31:0] avg_r;      // ((rs1+rs2+1) >>> 1)

    wire [31:0] brev_i = brev_comb(cmd_inputs_0);

    wire signed [32:0] cap_sum = {cmd_inputs_0[31], cmd_inputs_0} + {cmd_inputs_1[31], cmd_inputs_1};
    wire signed [32:0] cap_dif = {cmd_inputs_0[31], cmd_inputs_0} - {cmd_inputs_1[31], cmd_inputs_1};
    wire signed [32:0] cap_avg = {cmd_inputs_0[31], cmd_inputs_0} + {cmd_inputs_1[31], cmd_inputs_1} + 33'sd1;

    assign cmd_ready = !busy;
    wire capture = cmd_valid && cmd_ready;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            busy  <= 1'b0;
            v1    <= 1'b0;
            v2    <= 1'b0;
            v3    <= 1'b0;
            v4    <= 1'b0;
            fid_r <= 10'd0;
            a_r   <= 32'd0;
            b_r   <= 32'd0;
            sat_add_r <= 32'sd0;
            sat_sub_r <= 32'sd0;
            bmag_r    <= 32'sd0;
            nbmag_r   <= 32'sd0;
            avg_r     <= 32'd0;
        end else begin
            if (capture)                 busy <= 1'b1;
            else if (rsp_valid && rsp_ready) busy <= 1'b0;

            v1 <= capture;
            v2 <= v1;
            v3 <= v2;
            v4 <= v3;

            if (capture) begin
                fid_r <= cmd_function_id;
                a_r   <= cmd_inputs_0;
                b_r   <= cmd_inputs_1;
                // bit metrics
                brev_r <= brev_i;
                clz_r  <= clz_comb(cmd_inputs_0);
                ctz_r  <= clz_comb(brev_i);
                pcnt_r <= pc32_comb(cmd_inputs_0);
                ham_r  <= pc32_comb(cmd_inputs_0 ^ cmd_inputs_1);


                rnd_sum_r <= cmd_inputs_0 +
                             ((cmd_inputs_1[4:0] == 5'd0)
                                ? 32'd0
                                : (32'd1 << (cmd_inputs_1[4:0] - 5'd1)));
                dn_rnd_r  <= (cmd_function_id[8:4] == 5'd0)
                                ? 32'd0
                                : (32'd1 << (cmd_function_id[8:4] - 5'd1));


                sat_add_r <= (cap_sum[32] != cap_sum[31])
                             ? (cap_sum[32] ? -32'sd2147483648 : 32'sd2147483647)
                             : cap_sum[31:0];
                sat_sub_r <= (cap_dif[32] != cap_dif[31])
                             ? (cap_dif[32] ? -32'sd2147483648 : 32'sd2147483647)
                             : cap_dif[31:0];
                bmag_r    <= cmd_inputs_1[31] ? -$signed(cmd_inputs_1) : $signed(cmd_inputs_1);
                nbmag_r   <= cmd_inputs_1[31] ? $signed(cmd_inputs_1) : -$signed(cmd_inputs_1);
                avg_r     <= cap_avg[32:1];
            end
        end
    end

    wire [2:0] f3 = fid_r[2:0];
    wire [2:0] f7 = fid_r[5:3];    // sub-op for packed ops


    wire signed [15:0] p0 = sxt8(a_r[7:0])   * sxt8(b_r[7:0]);    wire signed [15:0] p1 = sxt8(a_r[15:8])  * sxt8(b_r[15:8]);
    wire signed [15:0] p2 = sxt8(a_r[23:16]) * sxt8(b_r[23:16]);
    wire signed [15:0] p3 = sxt8(a_r[31:24]) * sxt8(b_r[31:24]);
    wire signed [31:0] dot4_i = {{16{p0[15]}}, p0} + {{16{p1[15]}}, p1}
                              + {{16{p2[15]}}, p2} + {{16{p3[15]}}, p3};


    // The unsigned |a-b| version failed on hardware

    wire signed [8:0] d0 = $signed(a_r[7:0])    - $signed(b_r[7:0]);
    wire signed [8:0] d1 = $signed(a_r[15:8])   - $signed(b_r[15:8]);
    wire signed [8:0] d2 = $signed(a_r[23:16])  - $signed(b_r[23:16]);
    wire signed [8:0] d3 = $signed(a_r[31:24])  - $signed(b_r[31:24]);
    wire [8:0] ad0 = d0[8] ? -d0 : d0;
    wire [8:0] ad1 = d1[8] ? -d1 : d1;
    wire [8:0] ad2 = d2[8] ? -d2 : d2;
    wire [8:0] ad3 = d3[8] ? -d3 : d3;
    wire [31:0] sad4_i = {23'd0, ad0} + {23'd0, ad1} + {23'd0, ad2} + {23'd0, ad3};


    reg signed [31:0] dot4_r;
    reg        [31:0] sad4_r;
    reg signed [31:0] dn_sum_r;                                       

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            dot4_r   <= 32'sd0;
            sad4_r   <= 32'd0;
            dn_sum_r <= 32'sd0;
        end else if (v1) begin
            dot4_r   <= dot4_i;
            sad4_r   <= sad4_i;
            dn_sum_r <= dot4_i + dn_rnd_r;
        end
    end


    wire signed [15:0] ca_r = a_r[15:0];
    wire signed [15:0] ca_i = a_r[31:16];
    wire signed [15:0] cb_r = b_r[15:0];
    wire signed [15:0] cb_i = b_r[31:16];
    wire signed [16:0] ca_s = ca_r + ca_i;
    wire signed [16:0] cb_s = cb_r + cb_i;

    reg  signed [31:0] cm_p1_r, cm_p2_r;
    reg  signed [33:0] cm_p3_r;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            cm_p1_r <= 32'sd0;
            cm_p2_r <= 32'sd0;
            cm_p3_r <= 34'sd0;
        end else if (v1) begin
            cm_p1_r <= ca_r * cb_r;
            cm_p2_r <= ca_i * cb_i;
            cm_p3_r <= ca_s * cb_s;
        end
    end

    wire signed [32:0] cm_re = cm_p1_r - cm_p2_r;
    wire signed [34:0] cm_im = cm_p3_r - cm_p1_r - cm_p2_r;


    reg [31:0] cmul_r;
    reg [31:0] cmac_r;
    reg signed [32:0] cacc_re, cacc_im;

    wire signed [34:0] cm_re_w   = cm_re;
    wire signed [34:0] cm_im_w   = cm_im;
    wire signed [34:0] cacc_re_w = cacc_re;
    wire signed [34:0] cacc_im_w = cacc_im;
    wire signed [34:0] csum_re   = cacc_re_w + cm_re_w;
    wire signed [34:0] csum_im   = cacc_im_w + cm_im_w;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            cmul_r  <= 32'd0;
            cmac_r  <= 32'd0;
            cacc_re <= 33'sd0;
            cacc_im <= 33'sd0;
        end else if (v2) begin
            cmul_r <= {sat_q15(cm_im_w), sat_q15(cm_re_w)};
            case (fid_r[6:3])
                4'd7: begin  // CMAC16 : cacc += (a*b), rd = sat(new cacc)
                    cacc_re <= csum_re[32:0];
                    cacc_im <= csum_im[32:0];
                    cmac_r  <= {sat_q15(csum_im), sat_q15(csum_re)};
                end
                4'd8: begin  // CCLR : cacc = 0, rd = 0
                    cacc_re <= 33'sd0;
                    cacc_im <= 33'sd0;
                    cmac_r  <= 32'd0;
                end
                default: ;
            endcase
        end
    end


    wire signed [8:0] add_i0 = {a_r[7],a_r[7:0]}    + {b_r[7],b_r[7:0]};
    wire signed [8:0] add_i1 = {a_r[15],a_r[15:8]}  + {b_r[15],b_r[15:8]};
    wire signed [8:0] add_i2 = {a_r[23],a_r[23:16]} + {b_r[23],b_r[23:16]};
    wire signed [8:0] add_i3 = {a_r[31],a_r[31:24]} + {b_r[31],b_r[31:24]};

    wire signed [8:0] sub_i0 = {a_r[7],a_r[7:0]}    - {b_r[7],b_r[7:0]};
    wire signed [8:0] sub_i1 = {a_r[15],a_r[15:8]}  - {b_r[15],b_r[15:8]};
    wire signed [8:0] sub_i2 = {a_r[23],a_r[23:16]} - {b_r[23],b_r[23:16]};
    wire signed [8:0] sub_i3 = {a_r[31],a_r[31:24]} - {b_r[31],b_r[31:24]};

    wire [7:0] max_i0 = ($signed(a_r[7:0])    > $signed(b_r[7:0]))    ? a_r[7:0]    : b_r[7:0];
    wire [7:0] max_i1 = ($signed(a_r[15:8])   > $signed(b_r[15:8]))   ? a_r[15:8]   : b_r[15:8];
    wire [7:0] max_i2 = ($signed(a_r[23:16])  > $signed(b_r[23:16]))  ? a_r[23:16]  : b_r[23:16];
    wire [7:0] max_i3 = ($signed(a_r[31:24])  > $signed(b_r[31:24]))  ? a_r[31:24]  : b_r[31:24];

    wire [7:0] min_i0 = ($signed(a_r[7:0])    < $signed(b_r[7:0]))    ? a_r[7:0]    : b_r[7:0];
    wire [7:0] min_i1 = ($signed(a_r[15:8])   < $signed(b_r[15:8]))   ? a_r[15:8]   : b_r[15:8];
    wire [7:0] min_i2 = ($signed(a_r[23:16])  < $signed(b_r[23:16]))  ? a_r[23:16]  : b_r[23:16];
    wire [7:0] min_i3 = ($signed(a_r[31:24])  < $signed(b_r[31:24]))  ? a_r[31:24]  : b_r[31:24];

    wire [7:0] abs_i0 = a_r[7]   ? -a_r[7:0]   : a_r[7:0];
    wire [7:0] abs_i1 = a_r[15]  ? -a_r[15:8]  : a_r[15:8];
    wire [7:0] abs_i2 = a_r[23]  ? -a_r[23:16] : a_r[23:16];
    wire [7:0] abs_i3 = a_r[31]  ? -a_r[31:24] : a_r[31:24];


    wire signed [16:0] add16_i0 = {a_r[15],a_r[15:0]} + {b_r[15],b_r[15:0]};
    wire signed [16:0] add16_i1 = {a_r[31],a_r[31:16]} + {b_r[31],b_r[31:16]};
    wire signed [16:0] sub16_i0 = {a_r[15],a_r[15:0]} - {b_r[15],b_r[15:0]};
    wire signed [16:0] sub16_i1 = {a_r[31],a_r[31:16]} - {b_r[31],b_r[31:16]};


    reg [31:0] simd_r;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            simd_r <= 32'd0;
        end else if (v2) begin
            case (f7)
                3'd0: simd_r <= dot4_r;
                3'd1: simd_r <= sad4_r;
                3'd2: simd_r <= {sat_u8(add_i3), sat_u8(add_i2), sat_u8(add_i1), sat_u8(add_i0)};
                3'd3: simd_r <= {sat_u8(sub_i3), sat_u8(sub_i2), sat_u8(sub_i1), sat_u8(sub_i0)};
                3'd4: simd_r <= {max_i3, max_i2, max_i1, max_i0};
                3'd5: simd_r <= {min_i3, min_i2, min_i1, min_i0};
                3'd6: simd_r <= {abs_i3, abs_i2, abs_i1, abs_i0};
                3'd7: simd_r <= {a_r[7:0], a_r[15:8], a_r[23:16], a_r[31:24]};
            endcase
        end
    end


    reg [31:0] ext_r;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            ext_r <= 32'd0;
        end else if (v2) begin
            case (f7)
                3'd0: ext_r <= {sat_q15(add16_i1), sat_q15(add16_i0)};
                3'd1: ext_r <= {sat_q15(sub16_i1), sat_q15(sub16_i0)};
                3'd2: ext_r <= {{8{a_r[23]}}, a_r[23:16],
                                {8{a_r[7]}},  a_r[7:0]};
                3'd3: ext_r <= {b_r[31:16], a_r[15:0]};
                default: ext_r <= 32'd0;
            endcase
        end
    end


    reg signed [31:0] acc;
    reg        [31:0] mac_r;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            acc   <= 32'sd0;
            mac_r <= 32'd0;
        end else if (v2) begin
            case (fid_r[6:3])
                4'd0: begin acc <= acc + dot4_r; mac_r <= acc + dot4_r; end
                4'd1: begin acc <= acc + dot4_r; mac_r <= acc + dot4_r; end
                4'd2: begin acc <= dot4_r;       mac_r <= dot4_r;       end
                4'd3: begin                      mac_r <= acc;          end  // RDACC
                4'd4: begin acc <= 32'sd0;       mac_r <= 32'd0;        end  // ZRACC
                4'd5: begin acc <= acc + sad4_r; mac_r <= acc + sad4_r; end
                default:   mac_r <= acc;
            endcase
        end
    end


    reg [31:0] bit_r;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            bit_r <= 32'd0;
        end else if (v2) begin
            case (f7)
                3'd0: bit_r <= brev_r;
                3'd1: bit_r <= {26'd0, clz_r};
                3'd2: bit_r <= {26'd0, ctz_r};
                3'd3: bit_r <= {26'd0, pcnt_r};
                3'd4: bit_r <= {{24{a_r[7]}}, a_r[7:0]};
                3'd5: bit_r <= a_r[31] ? -a_r : a_r;
                3'd6: bit_r <= ($signed(a_r) > $signed(b_r)) ? a_r : b_r;
                3'd7: bit_r <= ($signed(a_r) < $signed(b_r)) ? a_r : b_r;
            endcase
        end
    end


    wire [4:0]          rsh = b_r[4:0];


    wire signed [31:0]  rnd_shift = (rsh == 5'd0) ? $signed(a_r)
                                                  : ($signed(rnd_sum_r) >>> rsh);

    wire signed [31:0]  clip_v = ($signed(a_r) > bmag_r)  ? bmag_r
                               : ($signed(a_r) < nbmag_r) ? nbmag_r
                               : $signed(a_r);


    wire [4:0]          shr_n = fid_r[8:4];
    wire signed [31:0]  dot_norm = (shr_n == 5'd0) ? dot4_r
                                                   : (dn_sum_r >>> shr_n);

    reg [31:0] misc_r;
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            misc_r <= 32'd0;
        end else if (v2) begin
            case (f7)
                3'd0: misc_r <= sat_add_r;
                3'd1: misc_r <= sat_sub_r;
                3'd2: misc_r <= rnd_shift;
                3'd3: misc_r <= clip_v;
                3'd4: misc_r <= avg_r;
                3'd5: misc_r <= dot_norm;
                3'd6: misc_r <= {26'd0, ham_r};
                3'd7: misc_r <= {16'd0, brev_r[31:16]};
            endcase
        end
    end


    wire        ext_sel = |fid_r[9:7];

    reg [31:0] sel_c;
    always @* begin
        if (ext_sel) begin
            sel_c = ext_r;
        end else case (fid_r[2:0])
                3'd0: sel_c = simd_r;


                3'd1: sel_c = (fid_r[6:3] == 4'd6) ? cmul_r
                            : (fid_r[6:3] == 4'd7) ? cmac_r
                            : (fid_r[6:3] == 4'd8) ? 32'd0
                            : mac_r;
                3'd2: sel_c = bit_r;
                3'd3: sel_c = misc_r;
                default: sel_c = 32'hDEAD_BEEF;
        endcase
    end

    reg [31:0] sel_r;
    always @(posedge clk or posedge reset) begin
        if (reset) sel_r <= 32'd0;
        else if (v3) sel_r <= sel_c;
    end

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            rsp_valid     <= 1'b0;
            rsp_outputs_0 <= 32'd0;
        end else begin
            if (v4) begin

                // instruction's answer while v4 is high.
                rsp_outputs_0 <= sel_r;
                rsp_valid     <= 1'b1;
            end else if (rsp_valid && rsp_ready)
                rsp_valid     <= 1'b0;
        end
    end

endmodule
