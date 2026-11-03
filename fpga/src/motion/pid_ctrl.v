/*
 * pid_ctrl.v
 * 定点PID速度调节器，按离散控制周期更新并对输出限幅。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module pid_ctrl #(

    parameter signed [15:0] OUT_MAX = 16'sd1000,
    parameter signed [15:0] OUT_MIN = -16'sd1000
)(
    input                    Clk,
    input                    Rst_n,

    input                    Enable,       
    input                    Clr,          
    input                    Tick,         

    input      signed [15:0] KpQ10,
    input      signed [15:0] KiQ10,
    input      signed [15:0] KdQ10,

    input      signed [15:0] Setpoint,     
    input      signed [15:0] Feedback,     

    output reg signed [15:0] Out,          
    output     signed [15:0] ErrOut,       
    output reg               Sat           
);

    localparam signed [15:0] OUT_HI = OUT_MAX;
    localparam signed [15:0] OUT_LO = OUT_MIN;

    localparam signed [37:0] OUT_HI_W = OUT_HI;
    localparam signed [37:0] OUT_LO_W = OUT_LO;

    wire signed [16:0] sp_ext = {Setpoint[15], Setpoint};
    wire signed [16:0] fb_ext = {Feedback[15], Feedback};
    wire signed [16:0] e_now  = sp_ext - fb_ext;       

    assign ErrOut = e_now[15:0];

    localparam P_IDLE = 2'd0,    
               P_DIFF = 2'd1,    
               P_MUL  = 2'd2,    
               P_ACC  = 2'd3;    

    reg [1:0] pst;

    reg signed [16:0] e0_r, e1_r, e2_r;

    reg signed [17:0] d1_r;                 
    reg signed [18:0] d2_r;                 

    reg signed [47:0] tp_r, ti_r, td_r;     
    reg signed [47:0] sum_r;                

    reg signed [15:0] u_acc;                

    wire signed [37:0] du_raw = sum_r[47:10];

    localparam signed [37:0] DU_MAX = 38'sd20;

    wire signed [37:0] du = (du_raw >  DU_MAX) ?  DU_MAX :
                            (du_raw < -DU_MAX) ? -DU_MAX : du_raw;

    wire signed [37:0] u_next = {{22{u_acc[15]}}, u_acc} + du;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        pst   <= P_IDLE;
        e0_r  <= 17'sd0;
        e1_r  <= 17'sd0;
        e2_r  <= 17'sd0;
        d1_r  <= 18'sd0;
        d2_r  <= 19'sd0;
        tp_r  <= 48'sd0;
        ti_r  <= 48'sd0;
        td_r  <= 48'sd0;
        sum_r <= 48'sd0;
        u_acc <= 16'sd0;
        Out   <= 16'sd0;
        Sat   <= 1'b0;
    end
    else if(Clr || !Enable) begin
        pst   <= P_IDLE;
        e0_r  <= 17'sd0;
        e1_r  <= 17'sd0;
        e2_r  <= 17'sd0;
        d1_r  <= 18'sd0;
        d2_r  <= 19'sd0;
        sum_r <= 48'sd0;
        u_acc <= 16'sd0;
        Out   <= 16'sd0;
        Sat   <= 1'b0;
    end
    else begin
        case(pst)

        P_IDLE: begin
            Sat <= 1'b0;
            if(Tick) begin
                e2_r <= e1_r;
                e1_r <= e0_r;
                e0_r <= e_now;
                pst  <= P_DIFF;
            end
        end

        P_DIFF: begin
            d1_r <= e0_r - e1_r;
            d2_r <= e0_r - (e1_r <<< 1) + e2_r;
            pst  <= P_MUL;
        end

        P_MUL: begin
            tp_r <= d1_r * KpQ10;
            ti_r <= e0_r * KiQ10;
            td_r <= d2_r * KdQ10;
            pst  <= P_ACC;
        end

        P_ACC: begin
            sum_r <= tp_r + ti_r + td_r;

            if(u_next > OUT_HI_W) begin
                u_acc <= OUT_HI;
                Sat   <= 1'b1;
            end
            else if(u_next < OUT_LO_W) begin
                u_acc <= OUT_LO;
                Sat   <= 1'b1;
            end
            else begin
                u_acc <= u_next[15:0];
                Sat   <= 1'b0;
            end

            Out <= u_acc;
            pst <= P_IDLE;
        end

        default: pst <= P_IDLE;
        endcase
    end

endmodule
