/*
 * att_align.v
 * IMU坐标系到车体坐标系的姿态映射。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module att_align (
    input  signed [15:0] ImuRoll,       
    input  signed [15:0] ImuPitch,      
    input  signed [15:0] ImuYaw,        
    output signed [15:0] AngRoll,       
    output signed [15:0] AngPitch,      
    output signed [15:0] AngYaw         
);

    assign AngRoll  = (`ATT_ROLL_SIGN  > 0) ? ImuRoll  : -ImuRoll;
    assign AngPitch = (`ATT_PITCH_SIGN > 0) ? ImuPitch : -ImuPitch;
    assign AngYaw   = (`ATT_YAW_SIGN   > 0) ? ImuYaw   : -ImuYaw;

endmodule
