/*
 * motor_drive.v
 * 左右电机方向和PWM驱动封装，连接闭环控制输出与物理引脚。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module motor_drive #(
    parameter CLK_FREQ    = `CLK_CTRL_HZ,
    parameter PWM_FREQ_HZ = `PWM_FREQ_HZ,
    parameter PWM_STEPS   = `PWM_STEPS
)(
    input                    Clk,
    input                    Rst_n,

    input      [7:0]         DirCmd,        
    input      [7:0]         SpeedPct,      

    input                    ClosdLoopEn,   
    input      signed [15:0] DutyL,         
    input      signed [15:0] DutyR,         

    output     [3:0]         MotorPwm,      
    output     [3:0]         MotorDir,      
    output     [3:0]         MotorDirN,     
    output signed [15:0]     DutyOutL,      
    output signed [15:0]     DutyOutR       
);

    assign MotorDirN = ~MotorDir;

    wire [7:0]  spd_lim    = (SpeedPct > 8'd100) ? 8'd100 : SpeedPct;
    wire [15:0] spd_mil_c  = spd_lim * 8'd10;        
    reg  [15:0] spd_mil;                             

    reg  [31:0] third_pr;
    wire [15:0] spd_third = third_pr[25:10];

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        spd_mil  <= 16'd0;
        third_pr <= 32'd0;
    end else begin
        spd_mil  <= spd_mil_c;
        third_pr <= spd_mil_c * 32'd341;
    end

    reg signed [15:0] mix_m0, mix_m1, mix_m2, mix_m3;    

    always @(*) begin
        case(DirCmd)
        `DIR_FWD:    begin mix_m0 =  spd_mil;   mix_m1 =  spd_mil;   mix_m2 =  spd_mil;   mix_m3 =  spd_mil;   end
        `DIR_BACK:   begin mix_m0 = -spd_mil;   mix_m1 = -spd_mil;   mix_m2 = -spd_mil;   mix_m3 = -spd_mil;   end
        `DIR_LEFT:   begin mix_m0 =  spd_third; mix_m1 =  spd_mil;   mix_m2 =  spd_third; mix_m3 =  spd_mil;   end
        `DIR_RIGHT:  begin mix_m0 =  spd_mil;   mix_m1 =  spd_third; mix_m2 =  spd_mil;   mix_m3 =  spd_third; end
        `DIR_SPIN_L: begin mix_m0 = -spd_mil;   mix_m1 =  spd_mil;   mix_m2 = -spd_mil;   mix_m3 =  spd_mil;   end
        `DIR_SPIN_R: begin mix_m0 =  spd_mil;   mix_m1 = -spd_mil;   mix_m2 =  spd_mil;   mix_m3 = -spd_mil;   end
        default:     begin mix_m0 = 16'sd0;     mix_m1 = 16'sd0;     mix_m2 = 16'sd0;     mix_m3 = 16'sd0;     end
        endcase
    end

    wire signed [15:0] duty_m0 = ClosdLoopEn ? DutyL : mix_m0;
    wire signed [15:0] duty_m1 = ClosdLoopEn ? DutyR : mix_m1;
    wire signed [15:0] duty_m2 = ClosdLoopEn ? DutyL : mix_m2;
    wire signed [15:0] duty_m3 = ClosdLoopEn ? DutyR : mix_m3;

    wire signed [15:0] duty_m0f = `MOTOR_L_INVERT ? -duty_m0 : duty_m0;
    wire signed [15:0] duty_m2f = `MOTOR_L_INVERT ? -duty_m2 : duty_m2;
    wire signed [15:0] duty_m1f = `MOTOR_R_INVERT ? -duty_m1 : duty_m1;
    wire signed [15:0] duty_m3f = `MOTOR_R_INVERT ? -duty_m3 : duty_m3;

    assign DutyOutL = duty_m0f;
    assign DutyOutR = duty_m1f;

    pwm_gen #(
        .CLK_FREQ    (CLK_FREQ   ),
        .PWM_FREQ_HZ (PWM_FREQ_HZ),
        .PWM_STEPS   (PWM_STEPS  )
    ) u_pwm_m0 (
        .Clk (Clk), .Rst_n (Rst_n),
        .Duty (duty_m0f), .Pwm (MotorPwm[0]), .Dir (MotorDir[0])
    );

    pwm_gen #(
        .CLK_FREQ    (CLK_FREQ   ),
        .PWM_FREQ_HZ (PWM_FREQ_HZ),
        .PWM_STEPS   (PWM_STEPS  )
    ) u_pwm_m1 (
        .Clk (Clk), .Rst_n (Rst_n),
        .Duty (duty_m1f), .Pwm (MotorPwm[1]), .Dir (MotorDir[1])
    );

    pwm_gen #(
        .CLK_FREQ    (CLK_FREQ   ),
        .PWM_FREQ_HZ (PWM_FREQ_HZ),
        .PWM_STEPS   (PWM_STEPS  )
    ) u_pwm_m2 (
        .Clk (Clk), .Rst_n (Rst_n),
        .Duty (duty_m2f), .Pwm (MotorPwm[2]), .Dir (MotorDir[2])
    );

    pwm_gen #(
        .CLK_FREQ    (CLK_FREQ   ),
        .PWM_FREQ_HZ (PWM_FREQ_HZ),
        .PWM_STEPS   (PWM_STEPS  )
    ) u_pwm_m3 (
        .Clk (Clk), .Rst_n (Rst_n),
        .Duty (duty_m3f), .Pwm (MotorPwm[3]), .Dir (MotorDir[3])
    );

endmodule
