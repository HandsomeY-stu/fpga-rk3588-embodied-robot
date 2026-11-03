/*
 * despeckle_median5.v
 * 水平五点中值滤波；分级比较网络与像素有效信号配合，抑制孤立采集噪点。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
// 五输入中值网络分成两级，限制单级比较深度；像素有效信号须与寄存器延迟配套。
module median5_net_lo #(parameter W = 5) (
    input  [W-1:0] v0,
    input  [W-1:0] v1,
    input  [W-1:0] v2,
    input  [W-1:0] v3,
    input  [W-1:0] v4,
    output [W-1:0] c0,
    output [W-1:0] c1,
    output [W-1:0] c2,
    output [W-1:0] c3,
    output [W-1:0] c4
);

    wire g1a = (v0 > v1);
    wire [W-1:0] a0 = g1a ? v0 : v1;       
    wire [W-1:0] a1 = g1a ? v1 : v0;       
    wire g1b = (v3 > v4);
    wire [W-1:0] a3 = g1b ? v3 : v4;
    wire [W-1:0] a4 = g1b ? v4 : v3;
    wire [W-1:0] a2 = v2;

    wire g2 = (a2 > a4);
    wire [W-1:0] b2 = g2 ? a2 : a4;
    wire [W-1:0] b4 = g2 ? a4 : a2;
    wire [W-1:0] b0 = a0;
    wire [W-1:0] b1 = a1;
    wire [W-1:0] b3 = a3;

    wire g3a = (b2 > b3);
    assign c2 = g3a ? b2 : b3;
    assign c3 = g3a ? b3 : b2;
    wire g3b = (b1 > b4);
    assign c1 = g3b ? b1 : b4;
    assign c4 = g3b ? b4 : b1;
    assign c0 = b0;

endmodule

module median5_net_hi #(parameter W = 5) (
    input  [W-1:0] c0,
    input  [W-1:0] c1,
    input  [W-1:0] c2,
    input  [W-1:0] c3,
    input  [W-1:0] c4,
    output [W-1:0] med
);

    wire g4 = (c0 > c3);
    wire [W-1:0] d0 = g4 ? c0 : c3;
    wire [W-1:0] d3 = g4 ? c3 : c0;
    wire [W-1:0] d1 = c1;
    wire [W-1:0] d2 = c2;
    wire [W-1:0] d4 = c4;

    wire g5a = (d0 > d2);
    wire [W-1:0] e0 = g5a ? d0 : d2;
    wire [W-1:0] e2 = g5a ? d2 : d0;
    wire g5b = (d1 > d3);
    wire [W-1:0] e1 = g5b ? d1 : d3;
    wire [W-1:0] e3 = g5b ? d3 : d1;

    wire g6 = (e1 > e2);
    wire [W-1:0] f2 = g6 ? e2 : e1;        

    assign med = f2;

endmodule

module median5_rgb565_lo (
    input  [15:0] v0,
    input  [15:0] v1,
    input  [15:0] v2,
    input  [15:0] v3,
    input  [15:0] v4,
    output [15:0] c0,
    output [15:0] c1,
    output [15:0] c2,
    output [15:0] c3,
    output [15:0] c4
);

    wire [4:0] b0,b1,b2,b3,b4;

    median5_net_lo #(.W(5)) u_b (.v0(v0[4:0]), .v1(v1[4:0]), .v2(v2[4:0]),
                                 .v3(v3[4:0]), .v4(v4[4:0]),
                                 .c0(b0), .c1(b1), .c2(b2), .c3(b3), .c4(b4));

    assign c0 = {11'd0, b0};
    assign c1 = {11'd0, b1};
    assign c2 = {v2[15:5], b2};
    assign c3 = {11'd0, b3};
    assign c4 = {11'd0, b4};

endmodule

module median5_rgb565_hi (
    input  [15:0] c0,
    input  [15:0] c1,
    input  [15:0] c2,
    input  [15:0] c3,
    input  [15:0] c4,
    output [15:0] med
);

    wire [4:0] bm;

    median5_net_hi #(.W(5)) u_b (.c0(c0[4:0]), .c1(c1[4:0]), .c2(c2[4:0]),
                                 .c3(c3[4:0]), .c4(c4[4:0]), .med(bm));

    assign med = {c2[15:5], bm};

endmodule

module despeckle_median5 #(parameter LINE_PX = 1280) (
    input             PCLK,
    input             Rst_n,
    input             de,
    input      [15:0] din,
    output reg [15:0] dout,
    output reg        dvalid
);

    reg [15:0] d0, d1, d2, d3, d4;

    reg [15:0] mc0, mc1, mc2, mc3, mc4;

    reg [15:0] q1, q2, q3, q4;

    reg        v1, v2, v3, v4, vld_r;
    reg [15:0] med_r, q3_r;

    reg [11:0] px_in;       
    reg [11:0] o_cnt;       

    wire [15:0] lc0, lc1, lc2, lc3, lc4;
    wire [15:0] med;
    median5_rgb565_lo u_med_lo (
        .v0(d4), .v1(d3), .v2(d2), .v3(d1), .v4(d0),
        .c0(lc0), .c1(lc1), .c2(lc2), .c3(lc3), .c4(lc4)
    );
    median5_rgb565_hi u_med_hi (
        .c0(mc0), .c1(mc1), .c2(mc2), .c3(mc3), .c4(mc4), .med(med)
    );

    wire bypass = (o_cnt < 12'd2) || (o_cnt >= (LINE_PX[11:0] - 12'd2));

    always @(posedge PCLK or negedge Rst_n)
    if(!Rst_n) begin
        d0 <= 16'd0; d1 <= 16'd0; d2 <= 16'd0; d3 <= 16'd0; d4 <= 16'd0;
        mc0 <= 16'd0; mc1 <= 16'd0; mc2 <= 16'd0; mc3 <= 16'd0; mc4 <= 16'd0;
        q1 <= 16'd0; q2 <= 16'd0; q3 <= 16'd0; q4 <= 16'd0;
        v1 <= 1'b0;  v2 <= 1'b0;  v3 <= 1'b0;  v4 <= 1'b0;
        med_r <= 16'd0; q3_r <= 16'd0; vld_r <= 1'b0;
        px_in <= 12'd0; o_cnt <= 12'd0;
    end
    else begin

        q1 <= din;
        q2 <= q1;
        q3 <= q2;

        v1 <= de;
        v2 <= v1;
        v3 <= v2;
        v4 <= v3;

        mc0 <= lc0;
        mc1 <= lc1;
        mc2 <= lc2;
        mc3 <= lc3;
        mc4 <= lc4;

        med_r <= med;
        q3_r  <= q3;
        q4    <= q3_r;       
        vld_r <= v4;         
        if(vld_r)
            o_cnt <= (o_cnt == LINE_PX[11:0] - 12'd1) ? 12'd0 : (o_cnt + 12'd1);

        if(de) begin
            d0 <= din;
            if(px_in == 12'd0) begin

                d1 <= din;
                d2 <= din;
                d3 <= din;
                d4 <= din;
            end
            else begin
                d1 <= d0;
                d2 <= d1;
                d3 <= d2;
                d4 <= d3;
            end
            px_in <= (px_in == LINE_PX[11:0] - 12'd1) ? 12'd0 : (px_in + 12'd1);
        end
    end

    always @(*) begin
        if(bypass)
            dout = q4;       
        else
            dout = med_r;
        dvalid = vld_r;
    end

endmodule
