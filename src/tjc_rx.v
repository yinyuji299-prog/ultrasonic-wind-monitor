`timescale 1ns/1ps


`default_nettype none

module tjc_rx (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire        clr_cmd,
    input  wire [7:0]  byte_in,
    input  wire        byte_valid,

    output reg         got_fe,
    output reg         got_fd,
    output reg         got_a5,
    output reg  [7:0]  last_byte,
    output reg         last_valid
);

    wire is_cmd = (byte_in == 8'hA0) || (byte_in == 8'hA5) ||
                  (byte_in == 8'h5A) || (byte_in == 8'hAA) ||
                  (byte_in == 8'h55) || (byte_in == 8'hF0);

    always @(posedge clk) begin
        if (rst) begin
            got_fe <= 1'b0; got_fd <= 1'b0; got_a5 <= 1'b0;
            last_byte <= 8'd0; last_valid <= 1'b0;
        end else begin
            got_fe <= 1'b0;
            got_fd <= 1'b0;
            last_valid <= 1'b0;
            if (clr_cmd || !enable) got_a5 <= 1'b0;
            if (enable && byte_valid) begin
                last_byte  <= byte_in;
                last_valid <= 1'b1;
                if (byte_in == 8'hFE) got_fe <= 1'b1;
                if (byte_in == 8'hFD) got_fd <= 1'b1;
                if (is_cmd)           got_a5 <= 1'b1;
            end
        end
    end

endmodule

`default_nettype wire
