/*
 * odom_xy.v
 * 利用轮速增量和航向估计平面位姿，采用定点积分。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module odom_xy #(
    parameter CLK_FREQ   = `CLK_CTRL_HZ,
    parameter POS_LIMIT_CM = 30000         
)(
    input                    Clk,
    input                    Rst_n,
    input                    Tick,         

    input      signed [15:0] DistMm,      
    input      signed [15:0] Yaw,         

    output reg signed [15:0] CurX,        
    output reg signed [15:0] CurY         
);

    localparam signed [15:0] POS_HI = POS_LIMIT_CM;
    localparam signed [15:0] POS_LO = -POS_LIMIT_CM;

    localparam signed [31:0] POS_HI32 = POS_LIMIT_CM;
    localparam signed [31:0] POS_LO32 = -POS_LIMIT_CM;

    wire signed [16:0] yaw17 = {Yaw[15], Yaw};
    wire signed [15:0] sy, cy;

    sincos_lut u_sincos (
        .Ang (yaw17),
        .Sin (sy),
        .Cos (cy)
    );

    reg signed [47:0] px_pr, py_pr;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        px_pr <= 48'sd0;
        py_pr <= 48'sd0;
    end
    else if(Tick) begin
        px_pr <= {32'd0, DistMm} * cy;     
        py_pr <= {32'd0, DistMm} * sy;
    end

    wire signed [47:0] px_r = px_pr + 48'sd1024;     
    wire signed [47:0] py_r = py_pr + 48'sd1024;

    wire signed [31:0] dx_q4 = px_r[42:11];
    wire signed [31:0] dy_q4 = py_r[42:11];

    reg signed [31:0] x_acc, y_acc;         

    wire signed [31:0] x_next = x_acc + dx_q4;
    wire signed [31:0] y_next = y_acc + dy_q4;

    wire signed [47:0] x48 = {{16{x_next[31]}}, x_next};
    wire signed [47:0] y48 = {{16{y_next[31]}}, y_next};
    wire signed [47:0] x_cm_pr = x48 * 48'sd6554 + 48'sd524288;    
    wire signed [47:0] y_cm_pr = y48 * 48'sd6554 + 48'sd524288;

    wire signed [31:0] x_cm = x_cm_pr[47:20];       
    wire signed [31:0] y_cm = y_cm_pr[47:20];

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        x_acc <= 32'sd0;
        y_acc <= 32'sd0;
        CurX  <= 16'sd0;
        CurY  <= 16'sd0;
    end
    else if(Tick) begin
        x_acc <= x_next;
        y_acc <= y_next;

        if(x_cm > POS_HI32)      CurX <= POS_HI;
        else if(x_cm < POS_LO32) CurX <= POS_LO;
        else                     CurX <= x_cm[15:0];

        if(y_cm > POS_HI32)      CurY <= POS_HI;
        else if(y_cm < POS_LO32) CurY <= POS_LO;
        else                     CurY <= y_cm[15:0];
    end

endmodule
