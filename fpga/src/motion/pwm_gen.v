/*
 * pwm_gen.v
 * 可参数化PWM发生器，将占空比目标转换为周期性输出。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module pwm_gen #(
    parameter CLK_FREQ   = `CLK_CTRL_HZ,  
    parameter PWM_FREQ_HZ= `PWM_FREQ_HZ,  
    parameter PWM_STEPS  = `PWM_STEPS     
)(
    input                    Clk,
    input                    Rst_n,
    input      signed [15:0] Duty,        
    output reg               Pwm,         
    output reg               Dir          
);

    localparam integer PERIOD     = CLK_FREQ / PWM_FREQ_HZ;           
    localparam integer DUTY_SCALE = (PERIOD * 1024) / PWM_STEPS;      
    localparam integer CNT_W      = 16;                               

    localparam [CNT_W-1:0] LAST_C = PERIOD - 1;                       

    wire signed [15:0] DUTY_POS =  $signed({1'b0, PWM_STEPS[15:0]});  
    wire signed [15:0] DUTY_NEG = -$signed({1'b0, PWM_STEPS[15:0]});  

    wire signed [15:0] duty_lim;    
    wire        [15:0] mag;         
    wire               dir_new;     

    assign duty_lim = (Duty > DUTY_POS) ? DUTY_POS :
                      (Duty < DUTY_NEG) ? DUTY_NEG : Duty;

    assign mag     = duty_lim[15] ? (~duty_lim[15:0] + 16'd1) : duty_lim[15:0];
    assign dir_new = duty_lim[15];

    reg  [31:0] thr_pr;       
    wire [15:0] threshold = thr_pr[25:10];    

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) thr_pr <= 32'd0;
    else       thr_pr <= mag * DUTY_SCALE;

    reg [CNT_W-1:0] Cnt;

    localparam integer DEAD_CYC   = (CLK_FREQ / 1000000) * `PWM_DEAD_TIME_NS / 1000;   
    localparam [15:0]  DEAD_CYC_W = DEAD_CYC;    

    reg        pwm_r;
    reg        dir_r;
    reg        chg;                  
    reg [15:0] dead_cnt;

    wire dir_commit = chg & (dead_cnt == 16'd0);    

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) Cnt <= {CNT_W{1'b0}};
    else if(dir_commit)    Cnt <= {CNT_W{1'b0}};    
    else if(Cnt >= LAST_C) Cnt <= {CNT_W{1'b0}};
    else                   Cnt <= Cnt + 1'b1;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        pwm_r    <= 1'b0;
        dir_r    <= 1'b1;
        chg      <= 1'b0;
        dead_cnt <= 16'd0;
    end
    else if(chg) begin
        pwm_r <= 1'b0;                               
        if(dead_cnt == 16'd0) begin
            dir_r <= dir_new;                        
            chg   <= 1'b0;
        end
        else dead_cnt <= dead_cnt - 16'd1;
    end
    else if(dir_new != dir_r) begin
        pwm_r    <= 1'b0;                            
        dead_cnt <= DEAD_CYC_W;
        chg      <= 1'b1;
    end
    else begin
        pwm_r <= (Cnt < threshold);                  
    end

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        Pwm <= 1'b0;
        Dir <= 1'b1;
    end
    else begin
        Pwm <= pwm_r;
        Dir <= dir_r;
    end

endmodule
