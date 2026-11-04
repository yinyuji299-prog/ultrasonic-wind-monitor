`timescale 1ns/1ps


module sapphire_soc (
    input               pll_inst1_CLKOUT0,
    input               pll_inst1_CLKOUT1,
    input               pll_inst1_LOCKED,

    input       [3:0]   key,
    output      [7:0]   led_data,


    output              spi_sclk_o,
    output              spi_mosi_o,
    input               spi_miso_i,
    output              spi_cs_o,


    output              screen_txd,
    (* mark_debug = 1 *)    input               screen_rxd,      // 屏 TX → FPGA（D5/GPIOL_28）


    output              pwm_a_o,          // D0  PWM_0
    output              pwm_a_oe,         // D0  输出使能（恒 1，配 inout 满足弱下拉配置）
    input               pwm_a_o_in,       // D0  输入半边（未用，仅为拿到 weak pulldown）
    output              pwm_b_o,          // D34 PWM_1
    output              pwm_b_oe,         // D34 输出使能（恒 1）
    input               pwm_b_o_in,       // D34 输入半边（未用）
    output              ad_cs_0_o,        // D7
    output              ad_sck_0_o,       // D6
    input               ad_sdo_0_i,       // D4
    output              ad_cs_1_o,        // D35
    output              ad_sck_1_o,       // D33
    input               ad_sdo_1_i,       // D31

    output              da_sdin_0_o,      // D8
    output              da_sclk_0_o,      // D9
    output              da_sync_0_o,      // U27.11
    output              da_sdin_1_o,      // D19
    output              da_sclk_1_o,      // D17
    output              da_sync_1_o,      // D23

    output              fpga_txd,
    input               fpga_rxd,

    input               i2c_sda_in,
    output              i2c_sda_out,
    output              i2c_sda_oe,
    input               i2c_scl_in,
    output              i2c_scl_out,
    output              i2c_scl_oe,

    input               jtag_inst1_TCK,
    input               jtag_inst1_TDI,
    output              jtag_inst1_TDO,
    input               jtag_inst1_SEL,
    input               jtag_inst1_CAPTURE,
    input               jtag_inst1_SHIFT,
    input               jtag_inst1_UPDATE,
    input               jtag_inst1_RESET
);

wire        clk      = pll_inst1_CLKOUT0;  // 100 MHz system / SoC
wire        rst      = ~pll_inst1_LOCKED;  // SoC io_asyncReset is active-high


wire clk_samp = clk;


reg [7:0] por_sys  = 8'd0;
reg [7:0] por_samp = 8'd0;
always @(posedge clk)
    if (por_sys != 8'hFF)
        por_sys <= por_sys + 8'd1;
always @(posedge clk_samp)
    if (por_samp != 8'hFF)
        por_samp <= por_samp + 8'd1;
wire rst_sys  = (por_sys  < 8'h20);
wire rst_samp = (por_samp < 8'h20);

wire        io_systemReset;
wire [3:0]  gpio_write;
wire [3:0]  gpio_oe;

wire        axi_awvalid, axi_awready;
wire [31:0] axi_awaddr;
wire [7:0]  axi_awid;
wire [3:0]  axi_awregion;
wire [7:0]  axi_awlen;
wire [2:0]  axi_awsize;
wire [1:0]  axi_awburst;
wire        axi_awlock;
wire [3:0]  axi_awcache, axi_awqos;
wire [2:0]  axi_awprot;
wire        axi_wvalid, axi_wready;
wire [31:0] axi_wdata;
wire [3:0]  axi_wstrb;
wire        axi_wlast;
wire        axi_bvalid, axi_bready;
wire [7:0]  axi_bid;
wire [1:0]  axi_bresp;
wire        axi_arvalid, axi_arready;
wire [31:0] axi_araddr;
wire [7:0]  axi_arid;
wire [3:0]  axi_arregion;
wire [7:0]  axi_arlen;
wire [2:0]  axi_arsize;
wire [1:0]  axi_arburst;
wire        axi_arlock;
wire [3:0]  axi_arcache, axi_arqos;
wire [2:0]  axi_arprot;
wire        axi_rvalid, axi_rready;
wire [31:0] axi_rdata;
wire [7:0]  axi_rid;
wire [1:0]  axi_rresp;
wire        axi_rlast;

wire [15:0] apb_paddr;
wire        apb_psel, apb_penable, apb_pwrite;
wire [31:0] apb_pwdata;

wire        i2c_sda_w, i2c_scl_w;
wire        spi_sclk;
wire        spi_d0_oe, spi_d1_oe, spi_d2_oe, spi_d3_oe;
wire        spi_d0_w,  spi_d1_w,  spi_d2_w,  spi_d3_w;
wire        spi_ss;


(* mark_debug = 1 *) wire [15:0] adc_samp_a, adc_samp_b;
(* mark_debug = 1 *) wire        adc_samp_valid;
(* mark_debug = 1 *) wire [31:0] adc_pair_cnt;
(* mark_debug = 1 *) wire [15:0] adc_raw_a, adc_raw_b;


reg         aw_pend, w_pend, bvalid_r, rvalid_r;
reg  [7:0]  bid_r, rid_r;

assign axi_awready = ~aw_pend & ~bvalid_r;
assign axi_wready  = ~w_pend  & ~bvalid_r;
assign axi_arready = ~rvalid_r;
assign axi_bvalid  = bvalid_r;
assign axi_bid     = bid_r;
assign axi_bresp   = 2'b00;
assign axi_rvalid  = rvalid_r;
assign axi_rdata   = 32'h0;
assign axi_rid     = rid_r;
assign axi_rresp   = 2'b00;
assign axi_rlast   = 1'b1;

always @(posedge clk) begin
    if (rst) begin
        aw_pend  <= 1'b0;
        w_pend   <= 1'b0;
        bvalid_r <= 1'b0;
        rvalid_r <= 1'b0;
        bid_r    <= 8'h0;
        rid_r    <= 8'h0;
    end else begin
        if (axi_awvalid && axi_awready) begin
            aw_pend <= 1'b1;
            bid_r   <= axi_awid;
        end
        if (axi_wvalid && axi_wready && axi_wlast)
            w_pend <= 1'b1;
        if (aw_pend && w_pend && !bvalid_r) begin
            bvalid_r <= 1'b1;
            aw_pend  <= 1'b0;
            w_pend   <= 1'b0;
        end else if (axi_bvalid && axi_bready)
            bvalid_r <= 1'b0;

        if (axi_arvalid && axi_arready) begin
            rvalid_r <= 1'b1;
            rid_r    <= axi_arid;
        end else if (axi_rvalid && axi_rready)
            rvalid_r <= 1'b0;
    end
end


wire        apb_pready;
wire [31:0] apb_prdata;
wire        apb_pslverr;
wire        dai_frame_ready;
wire        meas_busy_w;
wire        adc_en;          // ADC 采样使能：仅采集窗口内为 1

// inout 两个口的输出使能：恒开，行为与普通 output 完全一样。


assign pwm_a_oe = 1'b1;
assign pwm_b_oe = 1'b1;

// DAC7811_Ctrl ×2 已移入 apb_periph。

//   start_ADDA 触发；放在顶层就只能由上电 init 管，做不了 TGC。

apb_periph #(
    .N      (256),
    .NW     (8),
    .ADDR_W (8)
) u_periph (
    .clk_sys    (clk),
    .rst_sys    (rst_sys),
    .clk_samp   (clk_samp),
    .rst_samp   (rst_samp),

    .PADDR      (apb_paddr),
    .PSEL       (apb_psel),
    .PENABLE    (apb_penable),
    .PWRITE     (apb_pwrite),
    .PWDATA     (apb_pwdata),
    .PREADY     (apb_pready),
    .PRDATA     (apb_prdata),
    .PSLVERROR  (apb_pslverr),

    .samp_valid (adc_samp_valid),
    .samp_a     (adc_samp_a),
    .samp_b     (adc_samp_b),
    .frame_ready(dai_frame_ready),

    .screen_txd (screen_txd),
    .screen_rxd (screen_rxd),
    .pwm_out1   (pwm_a_o),                                              
    .pwm_out2   (pwm_b_o),          // PWM_1 → D34
    .da_sync_0_o(da_sync_0_o),
    .da_sclk_0_o(da_sclk_0_o),
    .da_sdin_0_o(da_sdin_0_o),
    .da_sync_1_o(da_sync_1_o),
    .da_sclk_1_o(da_sclk_1_o),
    .da_sdin_1_o(da_sdin_1_o),
    .meas_busy  (meas_busy_w),
    .acq_en_o   (adc_en)
);


wire        cfu_cmd_valid, cfu_cmd_ready, cfu_rsp_valid, cfu_rsp_ready;
wire [9:0]  cfu_fid;
wire [31:0] cfu_in0, cfu_in1, cfu_out;

cfu_proc u_cfu (
    .clk            (clk),
    .reset          (rst),
    .cmd_valid      (cfu_cmd_valid),
    .cmd_ready      (cfu_cmd_ready),
    .cmd_function_id(cfu_fid),
    .cmd_inputs_0   (cfu_in0),
    .cmd_inputs_1   (cfu_in1),
    .rsp_valid      (cfu_rsp_valid),
    .rsp_ready      (cfu_rsp_ready),
    .rsp_outputs_0  (cfu_out)
);


wire unused_spi_mosi;
LTC2315_Ctrl #(
    .BITS     (14),
    .CLK_DIV  (1),      // SCLK = 25 MHz（tSCK 40ns，半周期 20ns）：t2 = 20ns = 0.5·tSCK ✓
    .CS_SETUP (1),
    .CS_HOLD  (1),
    .GAP      (6),      // 每帧 66 拍 = 660 ns -> 1.5152 MSPS（1.5M 无法整除）
    .DUAL     (1)
) u_spi_adc (
    .clk        (clk_samp),
    .rst        (rst_samp),
    .enable     (adc_en),
    .clk_div    (8'd0),
    .spi_sclk   (ad_sck_0_o),
    .spi_cs_n   (ad_cs_0_o),
    .spi_sclk_b (ad_sck_1_o),
    .spi_cs_n_b (ad_cs_1_o),
    .spi_mosi   (unused_spi_mosi),
    // 互发互收要求 CS 与 SDO 必须配对：
    //   cs_0/保持(A组) 对应 sdo_0 -> 逻辑 A
    //   cs_1/保持(B组) 对应 sdo_1 -> 逻辑 B
    // 分时下只有一路 CS 有效，若按基线那样交叉读，会读到另一路的高阻脚。
    .spi_miso_a (ad_sdo_0_i),
    .spi_miso_b (ad_sdo_1_i),
    .samp_a     (adc_samp_a),
    .samp_b     (adc_samp_b),
    .samp_valid (adc_samp_valid),
    .pair_cnt   (adc_pair_cnt),
    .raw_a      (adc_raw_a),
    .raw_b      (adc_raw_b)
);


EfxSapphireSoc u_soc (
    .io_systemClk                 (clk),
    .io_asyncReset                (rst),
    .io_systemReset               (io_systemReset),
    .userInterruptA               (1'b0),

    .cpu0_customInstruction_cmd_valid   (cfu_cmd_valid),
    .cpu0_customInstruction_cmd_ready   (cfu_cmd_ready),
    .cpu0_customInstruction_function_id (cfu_fid),
    .cpu0_customInstruction_inputs_0    (cfu_in0),
    .cpu0_customInstruction_inputs_1    (cfu_in1),
    .cpu0_customInstruction_rsp_valid   (cfu_rsp_valid),
    .cpu0_customInstruction_rsp_ready   (cfu_rsp_ready),
    .cpu0_customInstruction_outputs_0   (cfu_out),

    .jtagCtrl_tck                 (jtag_inst1_TCK),
    .jtagCtrl_tdi                 (jtag_inst1_TDI),
    .jtagCtrl_tdo                 (jtag_inst1_TDO),
    .jtagCtrl_enable              (jtag_inst1_SEL),
    .jtagCtrl_capture             (jtag_inst1_CAPTURE),
    .jtagCtrl_shift               (jtag_inst1_SHIFT),
    .jtagCtrl_update              (jtag_inst1_UPDATE),
    .jtagCtrl_reset               (jtag_inst1_RESET),

    .axiA_awvalid                 (axi_awvalid),
    .axiA_awready                 (axi_awready),
    .axiA_awaddr                  (axi_awaddr),
    .axiA_awid                    (axi_awid),
    .axiA_awregion                (axi_awregion),
    .axiA_awlen                   (axi_awlen),
    .axiA_awsize                  (axi_awsize),
    .axiA_awburst                 (axi_awburst),
    .axiA_awlock                  (axi_awlock),
    .axiA_awcache                 (axi_awcache),
    .axiA_awqos                   (axi_awqos),
    .axiA_awprot                  (axi_awprot),
    .axiA_wvalid                  (axi_wvalid),
    .axiA_wready                  (axi_wready),
    .axiA_wdata                   (axi_wdata),
    .axiA_wstrb                   (axi_wstrb),
    .axiA_wlast                   (axi_wlast),
    .axiA_bvalid                  (axi_bvalid),
    .axiA_bready                  (axi_bready),
    .axiA_bid                     (axi_bid),
    .axiA_bresp                   (axi_bresp),
    .axiA_arvalid                 (axi_arvalid),
    .axiA_arready                 (axi_arready),
    .axiA_araddr                  (axi_araddr),
    .axiA_arid                    (axi_arid),
    .axiA_arregion                (axi_arregion),
    .axiA_arlen                   (axi_arlen),
    .axiA_arsize                  (axi_arsize),
    .axiA_arburst                 (axi_arburst),
    .axiA_arlock                  (axi_arlock),
    .axiA_arcache                 (axi_arcache),
    .axiA_arqos                   (axi_arqos),
    .axiA_arprot                  (axi_arprot),
    .axiA_rvalid                  (axi_rvalid),
    .axiA_rready                  (axi_rready),
    .axiA_rdata                   (axi_rdata),
    .axiA_rid                     (axi_rid),
    .axiA_rresp                   (axi_rresp),
    .axiA_rlast                   (axi_rlast),
    .axiAInterrupt                (1'b0),

    .system_uart_0_io_txd         (fpga_txd),
    .system_uart_0_io_rxd         (fpga_rxd),

    .system_i2c_0_io_sda_write    (i2c_sda_w),
    .system_i2c_0_io_sda_read     (1'b1),
    .system_i2c_0_io_scl_write    (i2c_scl_w),
    .system_i2c_0_io_scl_read     (1'b1),

    .system_gpio_0_io_read        (key),
    .system_gpio_0_io_write       (gpio_write),
    .system_gpio_0_io_writeEnable (gpio_oe),

    .io_apbSlave_0_PADDR          (apb_paddr),
    .io_apbSlave_0_PSEL           (apb_psel),
    .io_apbSlave_0_PENABLE        (apb_penable),
    .io_apbSlave_0_PREADY         (apb_pready),
    .io_apbSlave_0_PWRITE         (apb_pwrite),
    .io_apbSlave_0_PWDATA         (apb_pwdata),
    .io_apbSlave_0_PRDATA         (apb_prdata),
    .io_apbSlave_0_PSLVERROR      (apb_pslverr),

    .system_spi_0_io_sclk_write          (spi_sclk),
    .system_spi_0_io_data_0_writeEnable  (spi_d0_oe),
    .system_spi_0_io_data_0_read         (1'b1),
    .system_spi_0_io_data_0_write        (spi_d0_w),
    .system_spi_0_io_data_1_writeEnable  (spi_d1_oe),
    .system_spi_0_io_data_1_read         (spi_miso_i),
    .system_spi_0_io_data_1_write        (spi_d1_w),
    .system_spi_0_io_data_2_writeEnable  (spi_d2_oe),
    .system_spi_0_io_data_2_read         (1'b1),
    .system_spi_0_io_data_2_write        (spi_d2_w),
    .system_spi_0_io_data_3_writeEnable  (spi_d3_oe),
    .system_spi_0_io_data_3_read         (1'b1),
    .system_spi_0_io_data_3_write        (spi_d3_w),
    .system_spi_0_io_ss                  (spi_ss)
);


reg [26:0] hb;
always @(posedge clk) begin
    if (rst)
        hb <= 27'd0;
    else
        hb <= hb + 1'b1;
end

reg [24:0] samp_hb_led;
always @(posedge clk_samp) begin
    if (rst)
        samp_hb_led <= 25'd0;
    else
        samp_hb_led <= samp_hb_led + 1'b1;
end


assign i2c_sda_out = 1'b0;
assign i2c_sda_oe  = ~i2c_sda_w;
assign i2c_scl_out = 1'b0;
assign i2c_scl_oe  = ~i2c_scl_w;

wire [3:0] gpio_led = gpio_write & gpio_oe;
assign led_data[3:0] = ~gpio_led;
assign led_data[4]   = 1'b0;            // always ON  (bitstream loaded)
assign led_data[5]   = ~samp_hb_led[24]; // sample-domain heartbeat (CLKOUT1)
assign led_data[7:6] = ~hb[26:25];       // system-domain heartbeat


assign spi_sclk_o  = spi_sclk;
assign spi_mosi_o  = spi_d0_w;
assign spi_cs_o    = spi_ss;

endmodule
