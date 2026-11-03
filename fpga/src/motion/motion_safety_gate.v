/*
 * motion_safety_gate.v
 * 运动命令安全门：检查急停、命令时效和控制权限，输出最终放行信号。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module motion_safety_gate #(
    parameter integer CLK_FREQ           = `CLK_CTRL_HZ,
    parameter integer FRESH_TIMEOUT_MS   = 500,
    parameter integer STICKY_LOCAL       = 1,
    parameter integer REQ_LINK_ALIVE     = 0,
    parameter integer LINK_STOP_ON_UART  = 1
)(
    input             Clk,
    input             Rst_n,

    input             LocValid,
    input      [7:0]  LocCode,
    input      [15:0] LocArg0,
    input      [15:0] LocArg1,
    input      [15:0] LocArg2,

    input             UartValid,
    input      [7:0]  UartCode,
    input      [15:0] UartArg0,
    input      [15:0] UartArg1,
    input      [15:0] UartArg2,

    input             S1_Estop,         
    input             LinkLive,         

    output reg        OutValid,
    output reg [7:0]  OutCode,
    output reg [15:0] OutArg0,
    output reg [15:0] OutArg1,
    output reg [15:0] OutArg2,

    output reg        ForceStop,        
    output reg [3:0]  StopReason,       
    output reg [15:0] FreshMs,          
    output reg        LastSrcUart       
);

    localparam [3:0] SR_NONE    = 4'd0,    
                     SR_RESET   = 4'd1,    
                     SR_S1      = 4'd2,    
                     SR_STALE   = 4'd3,    
                     SR_NOLINK  = 4'd4,    
                     SR_CMDSTOP = 4'd5,    
                     SR_BLOCKED = 4'd6;    

    localparam integer MS_DIV = (CLK_FREQ / 1000);    

    reg        loc_pend, ua_pend;
    reg [7:0]  loc_c,  ua_c;
    reg [15:0] loc_a0, loc_a1, loc_a2;
    reg [15:0] ua_a0,  ua_a1,  ua_a2;

    wire        srv      = ua_pend | loc_pend;
    wire        srv_uart = ua_pend;
    wire [7:0]  srv_code = ua_pend ? ua_c  : loc_c;
    wire [15:0] srv_a0   = ua_pend ? ua_a0 : loc_a0;
    wire [15:0] srv_a1   = ua_pend ? ua_a1 : loc_a1;
    wire [15:0] srv_a2   = ua_pend ? ua_a2 : loc_a2;

    wire srv_is_stop   = (srv_code == `CMD_STOP) ||
                         ((srv_code == `CMD_MOTION) && (srv_a0[7:0] == `DIR_STOP));
    wire srv_is_motion = (srv_code == `CMD_MOTION) || (srv_code == `CMD_SPEED) ||
                         (srv_code == `CMD_ROTATE);
    wire srv_go        = srv_is_motion && !srv_is_stop;    

    wire srv_is_cfg    = (srv_code == `CMD_MODE) || (srv_code == `CMD_PID_SET);
    wire srv_is_nav    = (srv_code == `CMD_WAYPOINT) || (srv_code == `CMD_NAV_START) ||
                         (srv_code == `CMD_NAV_ABORT) ||
                         (srv_code == `CMD_SEG1_START) || (srv_code == `CMD_SEG1_ABORT);
    wire srv_is_passc  = srv_is_cfg | srv_is_nav;          

    reg  stop_latch;      
    reg  stale_armed;     
    reg  last_uart;       
    reg [3:0] latch_reason;    

    reg [15:0] ms_cnt;

    wire s1_now    = S1_Estop;
    wire stale_raw = stale_armed && (FreshMs >= FRESH_TIMEOUT_MS);
    wire link_used = (last_uart && (LINK_STOP_ON_UART != 0)) || (REQ_LINK_ALIVE != 0);
    wire link_now  = link_used && !LinkLive;

    wire can_clear = srv && (srv_go | (srv_is_passc && !srv_uart)) && !s1_now && !link_now;

    wire stale_now = stale_raw && !can_clear;

    wire blk_state = s1_now | stale_now | link_now | (stop_latch && !can_clear);

    reg  blk_state_d;
    wire stop_evt  = blk_state & ~blk_state_d;     

    wire ms_tick = (ms_cnt >= MS_DIV - 1);

    wire stop_first = stop_evt;                         

    wire allow_stop  = srv_is_stop;
    wire allow_passc = srv_is_passc & ~s1_now;               
    wire allow_go    = ~srv_is_stop & ~srv_is_passc & ~blk_state;

    wire pass       = srv & (allow_stop | allow_passc | allow_go) & ~stop_first;
    wire forced     = srv & ~pass & ~stop_first;        

    wire cons_uart  = (ua_pend  & ~stop_first);         
    wire cons_loc   = (~ua_pend & loc_pend & ~stop_first);

    wire emit_cmd   = pass & ~srv_is_stop;              
    wire emit_stop  = stop_first | forced | (pass & srv_is_stop);

    wire intents  = (latch_reason == SR_CMDSTOP) || (latch_reason == SR_S1);
    wire [3:0] reason_now = s1_now ? SR_S1 :
                            (stop_latch && intents) ? latch_reason :
                            link_now  ? SR_NOLINK :
                            stale_raw ? SR_STALE  : latch_reason;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        loc_pend <= 1'b0; ua_pend <= 1'b0;
        loc_c <= 8'd0; ua_c <= 8'd0;
        loc_a0 <= 16'd0; loc_a1 <= 16'd0; loc_a2 <= 16'd0;
        ua_a0  <= 16'd0; ua_a1  <= 16'd0; ua_a2  <= 16'd0;
        OutValid <= 1'b0; OutCode <= 8'd0;
        OutArg0 <= 16'd0; OutArg1 <= 16'd0; OutArg2 <= 16'd0;
        ForceStop <= 1'b1; StopReason <= SR_RESET; FreshMs <= 16'd0;
        LastSrcUart <= 1'b0;
        stop_latch <= 1'b1; stale_armed <= 1'b0; last_uart <= 1'b0;
        latch_reason <= SR_RESET;
        blk_state_d <= 1'b0; ms_cnt <= 16'd0;    
    end
    else begin

        if(LocValid) begin
            loc_pend <= 1'b1; loc_c <= LocCode;
            loc_a0 <= LocArg0; loc_a1 <= LocArg1; loc_a2 <= LocArg2;
        end
        if(UartValid) begin
            ua_pend <= 1'b1; ua_c <= UartCode;
            ua_a0 <= UartArg0; ua_a1 <= UartArg1; ua_a2 <= UartArg2;
        end

        OutValid <= 1'b0;

        if(stop_first) begin
            OutValid  <= 1'b1;
            OutCode   <= `CMD_MOTION;
            OutArg0   <= {8'd0, `DIR_STOP};       
            OutArg1   <= 16'd0;
            OutArg2   <= 16'd0;
            ForceStop <= 1'b1;

            if(s1_now)         latch_reason <= SR_S1;
            else if(link_now)  latch_reason <= SR_NOLINK;
            else if(stale_raw) latch_reason <= SR_STALE;
        end

        else if(pass) begin

            if(cons_uart && !UartValid) ua_pend  <= 1'b0;
            if(cons_loc  && !LocValid)  loc_pend <= 1'b0;    
            OutValid <= 1'b1;
            if(emit_cmd) begin
                OutCode <= srv_code; OutArg0 <= srv_a0;
                OutArg1 <= srv_a1;   OutArg2 <= srv_a2;

                stop_latch   <= can_clear ? 1'b0 : stop_latch;
                latch_reason <= can_clear ? SR_NONE : latch_reason;
                ForceStop    <= can_clear ? 1'b0 : ForceStop;
            end
            else begin                                 
                OutCode <= `CMD_MOTION;
                OutArg0 <= {8'd0, `DIR_STOP};
                OutArg1 <= 16'd0; OutArg2 <= 16'd0;
                stop_latch   <= 1'b1;
                latch_reason <= SR_CMDSTOP;
                ForceStop    <= 1'b1;
            end
        end

        else if(forced) begin
            if(cons_uart && !UartValid) ua_pend  <= 1'b0;
            if(cons_loc  && !LocValid)  loc_pend <= 1'b0;    
            OutValid  <= 1'b1;
            OutCode   <= `CMD_MOTION;
            OutArg0   <= {8'd0, `DIR_STOP};
            OutArg1   <= 16'd0; OutArg2 <= 16'd0;
            stop_latch<= 1'b1;
            ForceStop <= 1'b1;
        end

        StopReason <= blk_state ? reason_now : SR_NONE;

        if(pass && emit_cmd) begin
            FreshMs <= 16'd0;
            if(srv_uart) begin
                if(srv_go) begin
                    last_uart   <= 1'b1;
                    stale_armed <= 1'b1;
                    LastSrcUart <= 1'b1;
                end
            end
            else begin
                last_uart   <= 1'b0;
                stale_armed <= 1'b0;
                LastSrcUart <= 1'b0;
            end
        end
        else if(ms_tick && (FreshMs != 16'hFFFF)) begin
            FreshMs <= FreshMs + 16'd1;
        end

        ms_cnt <= ms_tick ? 16'd0 : (ms_cnt + 16'd1);

        blk_state_d <= blk_state;
    end

endmodule
