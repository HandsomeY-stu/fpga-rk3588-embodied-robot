/*
 * vision_follow_ctrl.v
 * FPGA视觉跟随接口控制器；与RK侧高级策略和命令安全门配合。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`timescale 1ns/1ps

`include "car_defines.vh"

module vision_follow_ctrl #(
    parameter CLK_FREQ     = `CLK_CTRL_HZ,
    parameter CX_CENTER    = 640,        
    parameter DEADBAND     = 40,         
    parameter NEAR_W       = 400,        
    parameter CONF_MIN     = 40,         
    parameter FWD_SPEED    = 8'd45,      
    parameter TURN_SPEED   = 8'd35,      
    parameter SLOW_SPEED   = 8'd20       
)(
    input                    Clk,
    input                    Rst_n,

    input                    VisValid,
    input  [7:0]             VisCls,
    input  [15:0]            VisCx,
    input  [15:0]            VisCy,
    input  [15:0]            VisW,
    input  [15:0]            VisH,
    input  [7:0]             VisConf,

    output reg               FollowValid,
    output reg [7:0]         FollowCode,
    output reg [15:0]        FollowArg0,

    output reg [3:0]         FsmState,
    output reg               TargetPresent
);

    wire target_present = VisValid && (VisCls != 8'd0) &&
                          (VisConf >= CONF_MIN) && (VisW > 0) && (VisH > 0);

    wire signed [15:0] cx_err = $signed(VisCx) - $signed(16'd640);    

    wire near_target = (VisW >= NEAR_W);

    wire centered = (cx_err >= -DEADBAND) && (cx_err <= DEADBAND);

    localparam [3:0] ST_IDLE  = 4'd0,    
                     ST_LEFT  = 4'd1,    
                     ST_RIGHT = 4'd2,    
                     ST_FWD   = 4'd3,    
                     ST_NEAR  = 4'd4;    

    reg [3:0] st;
    wire [3:0] st_next;
    wire [7:0] dir_next, spd_next;

    assign dir_next = (!target_present) ? `DIR_STOP :
                      (near_target)     ? `DIR_STOP :           
                      (cx_err < -DEADBAND) ? `DIR_LEFT  :
                      (cx_err >  DEADBAND) ? `DIR_RIGHT :
                                             `DIR_FWD;          

    assign spd_next = (!target_present) ? 8'd0 :
                      (near_target)     ? 8'd0 :               
                      (centered)        ? FWD_SPEED :
                                          TURN_SPEED;          

    assign st_next = (!target_present) ? ST_IDLE :
                     (near_target)     ? ST_NEAR :
                     (cx_err < -DEADBAND) ? ST_LEFT :
                     (cx_err >  DEADBAND) ? ST_RIGHT :
                                            ST_FWD;

    reg [7:0] last_dir;
    reg [7:0] last_spd;
    wire changed = (dir_next != last_dir) || (spd_next != last_spd);

    always @(posedge Clk or negedge Rst_n) begin
        if (!Rst_n) begin
            st          <= ST_IDLE;
            TargetPresent <= 1'b0;
            FollowValid <= 1'b0;
            FollowCode  <= `CMD_MOTION;
            FollowArg0  <= {`DIR_STOP, 8'd0};
            last_dir    <= `DIR_STOP;
            last_spd    <= 8'd0;
        end else begin
            st           <= st_next;
            TargetPresent <= target_present;
            FollowCode   <= `CMD_MOTION;

            if (changed) begin
                FollowValid <= 1'b1;
                FollowArg0  <= {dir_next, spd_next};
                last_dir    <= dir_next;
                last_spd    <= spd_next;
            end else begin
                FollowValid <= 1'b0;
            end
        end
    end

    always @(posedge Clk or negedge Rst_n)
        if (!Rst_n) FsmState <= ST_IDLE;
        else        FsmState <= st_next;

endmodule
