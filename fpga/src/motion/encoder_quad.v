/*
 * encoder_quad.v
 * 正交编码器采样与方向判定，输出带符号脉冲计数。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module encoder_quad #(
    parameter CLK_FREQ       = `CLK_CTRL_HZ,
    parameter CTRL_PERIOD_MS = `CTRL_PERIOD_MS,
    parameter ENC_CPR        = `ENC_CPR,
    parameter GEAR_RATIO     = `GEAR_RATIO,
    parameter ENC_QUAD       = `ENC_QUAD,
    parameter WHEEL_DIAM_MM  = `WHEEL_DIAM_MM,
    parameter INVERT         = 0           
)(
    input                    Clk,
    input                    Rst_n,

    input                    EncA,         
    input                    EncB,         

    output signed [31:0]     Count,        
    output signed [15:0]     Speed,        
    output reg               Dir,          
    output reg               PulseAlive    
);

    localparam integer CNT_PER_REV = ENC_CPR * GEAR_RATIO * ENC_QUAD;
    localparam integer MM_PER_REV  = (WHEEL_DIAM_MM * 31416) / 10000;    

    localparam integer SPEED_Q20   = ((MM_PER_REV << 20) / (CNT_PER_REV * CTRL_PERIOD_MS)) * 1000;
    localparam integer WINDOW_CNT  = (CLK_FREQ / 1000) * CTRL_PERIOD_MS;  

    reg [1:0] a_sync, b_sync;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        a_sync <= 2'b00;
        b_sync <= 2'b00;
    end
    else begin
        a_sync <= {a_sync[0], EncA};
        b_sync <= {b_sync[0], EncB};
    end

    wire a = a_sync[1];
    wire b = b_sync[1];

    reg pa, pb;

    reg signed [1:0] delta;
    always @(*) begin
        case({pa, pb, a, b})
        4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: delta =  2'sd1;    
        4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: delta = -2'sd1;    
        default:                                delta =  2'sd0;    
        endcase
    end

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        pa <= 1'b0;
        pb <= 1'b0;
    end
    else begin
        pa <= a;
        pb <= b;
    end

    reg [7:0] start_cnt;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) start_cnt <= 8'd0;
    else if(start_cnt != 8'hFF) start_cnt <= start_cnt + 1'b1;

    wire start_ok = (start_cnt == 8'hFF);

    wire signed [1:0] delta_g = start_ok ? delta : 2'sd0;

    reg signed [31:0] count_r;
    reg signed [15:0] win_cnt;         
    reg signed [15:0] win_cnt_lat;     
    reg [31:0]        win_timer;

    wire dir_raw = delta_g[1];         

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        count_r     <= 32'sd0;
        win_cnt     <= 16'sd0;
        win_cnt_lat <= 16'sd0;
        win_timer   <= 32'd0;
        PulseAlive  <= 1'b0;
    end
    else begin

        if(delta_g != 2'sd0)
            count_r <= count_r + {{30{delta_g[1]}}, delta_g};

        if(win_timer >= WINDOW_CNT - 1) begin
            win_timer   <= 32'd0;
            win_cnt_lat <= win_cnt;
            win_cnt     <= 16'sd0;

            PulseAlive  <= (win_cnt != 16'sd0);
        end
        else begin
            win_timer <= win_timer + 1'b1;
            if(delta_g != 2'sd0)
                win_cnt <= win_cnt + {{14{delta_g[1]}}, delta_g};
        end
    end

    reg signed [47:0] spd_pr;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) spd_pr <= 48'sd0;
    else       spd_pr <= win_cnt_lat * SPEED_Q20;

    wire signed [31:0] count_out = INVERT ? -count_r : count_r;
    wire signed [15:0] speed_out = INVERT ? -spd_pr[35:20] : spd_pr[35:20];

    assign Count = count_out;
    assign Speed = speed_out;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) Dir <= 1'b1;
    else if(win_cnt_lat != 16'sd0) Dir <= ~speed_out[15];

endmodule
