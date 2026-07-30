// Minimal DVI/HDMI transmitter for Gowin GW1NR-9C (Tang Nano 9K).
//
// TMDS 8b10b encoder logic adapted from Clifford Wolf's SVO project
// (ISC license): https://github.com/cliffordwolf/picorv32 / svo_tmds.v
//
// Serialization uses the Gowin OSER10 primitive (10:1 DDR serializer,
// FCLK must run at 5x PCLK) and ELVDS_OBUF for the pseudo-differential
// TMDS output pairs.
module dvi_tx (
    input  wire       clk_pixel,   // pixel clock (e.g. 74.25 MHz for 720p60)
    input  wire       clk_tmds,    // 5x pixel clock (serializer fast clock)
    input  wire       reset_n,

    input  wire [7:0] red,
    input  wire [7:0] green,
    input  wire [7:0] blue,
    input  wire       hsync,
    input  wire       vsync,
    input  wire       de,

    output wire        tmds_clk_p,
    output wire        tmds_clk_n,
    output wire [2:0]  tmds_d_p,
    output wire [2:0]  tmds_d_n
);

    wire [9:0] tmds_0, tmds_1, tmds_2;

    // Channel 0 (blue) carries hsync/vsync during blanking, per DVI spec.
    tmds_encoder enc0 (.clk(clk_pixel), .resetn(reset_n), .de(de), .ctrl({vsync, hsync}), .din(blue),  .dout(tmds_0));
    tmds_encoder enc1 (.clk(clk_pixel), .resetn(reset_n), .de(de), .ctrl(2'b00),          .din(green), .dout(tmds_1));
    tmds_encoder enc2 (.clk(clk_pixel), .resetn(reset_n), .de(de), .ctrl(2'b00),          .din(red),   .dout(tmds_2));

    wire [2:0] tmds_d0, tmds_d1, tmds_d2, tmds_d3, tmds_d4;
    wire [2:0] tmds_d5, tmds_d6, tmds_d7, tmds_d8, tmds_d9;
    wire [2:0] tmds_d;

    assign {tmds_d9[0], tmds_d8[0], tmds_d7[0], tmds_d6[0], tmds_d5[0],
            tmds_d4[0], tmds_d3[0], tmds_d2[0], tmds_d1[0], tmds_d0[0]} = tmds_0;
    assign {tmds_d9[1], tmds_d8[1], tmds_d7[1], tmds_d6[1], tmds_d5[1],
            tmds_d4[1], tmds_d3[1], tmds_d2[1], tmds_d1[1], tmds_d0[1]} = tmds_1;
    assign {tmds_d9[2], tmds_d8[2], tmds_d7[2], tmds_d6[2], tmds_d5[2],
            tmds_d4[2], tmds_d3[2], tmds_d2[2], tmds_d1[2], tmds_d0[2]} = tmds_2;

    OSER10 tmds_serdes [2:0] (
        .Q(tmds_d),
        .D0(tmds_d0), .D1(tmds_d1), .D2(tmds_d2), .D3(tmds_d3), .D4(tmds_d4),
        .D5(tmds_d5), .D6(tmds_d6), .D7(tmds_d7), .D8(tmds_d8), .D9(tmds_d9),
        .PCLK(clk_pixel),
        .FCLK(clk_tmds),
        .RESET(~reset_n)
    );

    ELVDS_OBUF tmds_bufds [3:0] (
        .I({clk_pixel, tmds_d}),
        .O({tmds_clk_p, tmds_d_p}),
        .OB({tmds_clk_n, tmds_d_n})
    );

endmodule

module tmds_encoder (
    input  wire       clk,
    input  wire       resetn,
    input  wire       de,
    input  wire [1:0] ctrl,
    input  wire [7:0] din,
    output reg  [9:0] dout
);

    function [3:0] n1;
        input [7:0] bits;
        integer i;
        begin
            n1 = 0;
            for (i = 0; i < 8; i = i + 1)
                n1 = n1 + bits[i];
        end
    endfunction

    function [3:0] n0;
        input [7:0] bits;
        integer i;
        begin
            n0 = 0;
            for (i = 0; i < 8; i = i + 1)
                n0 = n0 + !bits[i];
        end
    endfunction

    reg [9:0] dout_buf2, q_out, q_out_next;
    reg [3:0] n0_qm, n1_qm;
    reg signed [7:0] cnt, cnt_next, cnt_tmp;
    reg [8:0] qm;

    always @(posedge clk) begin
        if (!resetn) begin
            cnt   <= 0;
            q_out <= 0;
        end else if (!de) begin
            cnt <= 0;
            case (ctrl)
                2'b00: q_out <= 10'b1101010100;
                2'b01: q_out <= 10'b0010101011;
                2'b10: q_out <= 10'b0101010100;
                2'b11: q_out <= 10'b1010101011;
            endcase
        end else begin
            if ((n1(din) > 4) | ((n1(din) == 4) & (din[0] == 0))) begin
                qm[0] = din[0];
                qm[1] = qm[0] ^~ din[1];
                qm[2] = qm[1] ^~ din[2];
                qm[3] = qm[2] ^~ din[3];
                qm[4] = qm[3] ^~ din[4];
                qm[5] = qm[4] ^~ din[5];
                qm[6] = qm[5] ^~ din[6];
                qm[7] = qm[6] ^~ din[7];
                qm[8] = 1'b0;
            end else begin
                qm[0] = din[0];
                qm[1] = qm[0] ^ din[1];
                qm[2] = qm[1] ^ din[2];
                qm[3] = qm[2] ^ din[3];
                qm[4] = qm[3] ^ din[4];
                qm[5] = qm[4] ^ din[5];
                qm[6] = qm[5] ^ din[6];
                qm[7] = qm[6] ^ din[7];
                qm[8] = 1'b1;
            end

            n0_qm = n0(qm[7:0]);
            n1_qm = n1(qm[7:0]);

            if ((cnt == 0) | (n1_qm == n0_qm)) begin
                q_out_next[9]   = ~qm[8];
                q_out_next[8]   =  qm[8];
                q_out_next[7:0] = (qm[8] ? qm[7:0] : ~qm[7:0]);
                if (qm[8] == 0)
                    cnt_next = cnt + (n0_qm - n1_qm);
                else
                    cnt_next = cnt + (n1_qm - n0_qm);
            end else if (((cnt > 0) & (n1_qm > n0_qm)) | ((cnt < 0) & (n0_qm > n1_qm))) begin
                q_out_next[9]   = 1'b1;
                q_out_next[8]   = qm[8];
                q_out_next[7:0] = ~qm[7:0];
                cnt_tmp = cnt + (n0_qm - n1_qm);
                cnt_next = qm[8] ? (cnt_tmp + 2'h2) : cnt_tmp;
            end else begin
                q_out_next[9]   = 1'b0;
                q_out_next[8]   = qm[8];
                q_out_next[7:0] = qm[7:0];
                cnt_tmp = cnt + (n1_qm - n0_qm);
                cnt_next = qm[8] ? cnt_tmp : (cnt_tmp - 2'h2);
            end
            cnt   <= cnt_next;
            q_out <= q_out_next;
        end

        // extra pipeline stage gives synthesis some retiming slack
        dout_buf2 <= q_out;
        dout      <= dout_buf2;
    end
endmodule
