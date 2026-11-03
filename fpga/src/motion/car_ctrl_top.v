/*
 * car_ctrl_top.v
 * 运动子系统集成：命令解码、模式仲裁、避障/导航接管及状态输出。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module car_ctrl_top #(
    parameter CLK_FREQ      = `CLK_CTRL_HZ,
    parameter LOOP_EN_SEL   = 0,           
    parameter AVOID_EN      = 1,           
    parameter NAV_EN        = 0,           
    parameter NAV_ARRIVE_CM = 10,

    parameter SEG1_EN          = 0,
    parameter [7:0]  SEG1_CRUISE_SPEED = 8'd50,      

    parameter DBG_ENCDIR_EN = 0,

    parameter integer TRIM_L_Q10 = 1024,
    parameter integer TRIM_R_Q10 = 1024,

    parameter integer HDG_EN          = 0,
    parameter integer HDG_KP_Q8       = 64,
    parameter integer HDG_MAX_MMPS    = 150,
    parameter integer HDG_TOL_D100    = 20,
    parameter integer HDG_STALE_TICKS = 30
)(
    input                    Clk,
    input                    Rst_n,

    input                    CmdValid,
    input      [7:0]         CmdCode,
    input      [15:0]        CmdArg0,
    input      [15:0]        CmdArg1,
    input      [15:0]        CmdArg2,

    input      signed [15:0] AngRoll,
    input      signed [15:0] AngPitch,
    input      signed [15:0] AngYaw,
    input                    AngValid,
    input                    ImuLink,

    input                    LinkAlive,

    input                    EncA0, EncB0,        
    input                    EncA1, EncB1,        
    input                    UsEcho0,             
    input                    UsEcho1,             
    input                    UsEcho2,             
    input      [3:0]         Key,                 
    input      [7:0]         Dip,                 
    input                    SafetyStop,         

    output     [3:0]         MotorPwm,            
    output     [3:0]         MotorDir,            
    output     [3:0]         MotorDirN,           
    output                   UsTrig0,
    output                   UsTrig1,
    output                   UsTrig2,
    output                   Beep,                
    output     [3:0]         BLed,                

    output     [7:0]         StMode,
    output     [7:0]         StDir,
    output     signed [15:0] StDutyL,
    output     signed [15:0] StDutyR,
    output     signed [15:0] StVelL,
    output     signed [15:0] StVelR,
    output     signed [31:0] StOdomL,
    output     signed [31:0] StOdomR,
    output     [15:0]        StRangeF,
    output     [15:0]        StRangeL,
    output     [15:0]        StRangeR,
    output     [7:0]         StFault,
    output     [7:0]         StNavState,
    output     [7:0]         StNavWp,
    output     signed [15:0] StNavX,
    output     signed [15:0] StNavY,
    output     signed [15:0] StNavYaw,
    output     signed [15:0] StRemainCm,

    output     [7:0]         StAvoidState,    
    output                   StAvoidNear,     
    output                   StAvoidActive,   

    output     signed [15:0] StHdgCorr,       
    output                   StHdgAct,        

    output     [7:0]         StSeg1State,     
    output                   StSeg1Done,      
    output     signed [31:0] StSeg1DistMm,    
    output     [7:0]         StSeg1Err        

);

    wire signed [31:0] cnt_l, cnt_r;
    wire signed [15:0] vel_l, vel_r;
    wire               enc_live_l, enc_live_r;
    wire               enc_dir_l,  enc_dir_r;    

    wire [2:0]  us_start;
    wire [15:0] us_d0, us_d1, us_d2;
    wire        us_v0, us_v1, us_v2;
    wire [47:0] us_dist_in;
    wire [2:0]  us_valid_in;
    wire [47:0] us_dist_out;
    wire [2:0]  us_timeout, us_done;

    wire cmd_nav_start, cmd_nav_abort, cmd_wp_new;

    wire signed [16:0] v_avg;
    wire signed [15:0] dist_mm;
    wire signed [15:0] cur_x, cur_y;

    wire mode_avoid, mode_nav;

    wire [7:0] av_dir, av_spd;
    wire       av_active, av_near;
    wire [7:0] av_state;                        
    wire [7:0] nav_dir, nav_spd, nav_state, nav_wp;
    wire       nav_active;

    wire       ov_nav, Override;
    wire [7:0] OverDir, OverSpeed;

    wire        cmd_seg1_start, cmd_seg1_abort;
    wire        mode_seg1;
    wire [7:0]  s1_dir, s1_spd, s1_state, s1_err;
    wire        s1_act, s1_done;
    wire        s1_beep;           
    wire signed [31:0] s1_dist;
    wire        ov_s1;

    wire [7:0]         mc_dir, mc_spd;
    wire [7:0]         mc_eff_dir, mc_eff_spd;    
    wire signed [15:0] mc_dutyl, mc_dutyr;
    wire               mc_closed, mc_rotbusy, mc_estop, mc_nopulse;
    wire signed [31:0] mc_odoml, mc_odomr;

    wire signed [15:0] duty_out_l, duty_out_r;

    wire key_s1_sync, Estop;

    reg [31:0]        tick_cnt;
    reg [7:0]         mode_r;
    reg signed [15:0] wp_x, wp_y, wp_theta;
    reg signed [47:0] dm_pr;

    wire Tick = (tick_cnt >= `CTRL_PERIOD_CNT - 1);

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n)          tick_cnt <= 32'd0;
    else if(Tick)       tick_cnt <= 32'd0;
    else                tick_cnt <= tick_cnt + 1'b1;

    encoder_quad #(
        .INVERT (`ENC_L_INVERT)
    ) u_enc_l (
        .Clk (Clk), .Rst_n (Rst_n),
        .EncA (EncA0), .EncB (EncB0),
        .Count (cnt_l), .Speed (vel_l),
        .Dir (enc_dir_l), .PulseAlive (enc_live_l)
    );

    encoder_quad #(
        .INVERT (`ENC_R_INVERT)
    ) u_enc_r (
        .Clk (Clk), .Rst_n (Rst_n),
        .EncA (EncA1), .EncB (EncB1),
        .Count (cnt_r), .Speed (vel_r),
        .Dir (enc_dir_r), .PulseAlive (enc_live_r)
    );

    assign cmd_nav_start = CmdValid && (CmdCode == `CMD_NAV_START);
    assign cmd_nav_abort = CmdValid && (CmdCode == `CMD_NAV_ABORT);
    assign cmd_wp_new    = CmdValid && (CmdCode == `CMD_WAYPOINT);

    reg mode_seg1_d;
    always @(posedge Clk or negedge Rst_n)
        if(!Rst_n) mode_seg1_d <= 1'b0;
        else       mode_seg1_d <= mode_seg1;

    wire seg1_entry_evt = mode_seg1 & ~mode_seg1_d;       
    assign cmd_seg1_start = (CmdValid && (CmdCode == `CMD_SEG1_START)) || seg1_entry_evt;

    reg [23:0] seg1_beep_cnt;
    reg        seg1_beep_led;
    always @(posedge Clk or negedge Rst_n)
        if(!Rst_n) begin seg1_beep_cnt <= 24'd0; seg1_beep_led <= 1'b0; end
        else if(seg1_entry_evt) begin seg1_beep_cnt <= 24'd0; seg1_beep_led <= 1'b1; end
        else if(seg1_beep_led) begin

            if(seg1_beep_cnt >= 24'd10_000_000) seg1_beep_led <= 1'b0;
            else                                seg1_beep_cnt <= seg1_beep_cnt + 24'd1;
        end

    assign cmd_seg1_abort = CmdValid && (CmdCode == `CMD_SEG1_ABORT);

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        wp_x     <= 16'sd0;
        wp_y     <= 16'sd0;
        wp_theta <= 16'sd0;
    end
    else if(cmd_wp_new) begin
        wp_x     <= CmdArg0;       
        wp_y     <= CmdArg1;       
        wp_theta <= CmdArg2;       
    end

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) mode_r <= `MODE_MANUAL;
    else if(CmdValid && (CmdCode == `CMD_MODE)) mode_r <= CmdArg0[7:0];

    assign mode_avoid = (mode_r == `MODE_AVOID);
    assign mode_nav   = (mode_r == `MODE_NAV);
    assign mode_seg1  = (mode_r == `MODE_SEG1);      

    wire LoopEn = LOOP_EN_SEL ? Dip[0] : 1'b1;

    motion_ctrl #(
        .CLK_FREQ (CLK_FREQ),
        .TRIM_L_Q10 (TRIM_L_Q10),       
        .TRIM_R_Q10 (TRIM_R_Q10),

        .HDG_EN          (HDG_EN),
        .HDG_KP_Q8       (HDG_KP_Q8),
        .HDG_MAX_MMPS    (HDG_MAX_MMPS),
        .HDG_TOL_D100    (HDG_TOL_D100),
        .HDG_STALE_TICKS (HDG_STALE_TICKS)
    ) u_motion (
        .Clk (Clk), .Rst_n (Rst_n), .Tick (Tick),
        .CmdValid (CmdValid), .CmdCode (CmdCode),
        .CmdArg0 (CmdArg0), .CmdArg1 (CmdArg1), .CmdArg2 (CmdArg2),
        .VelL (vel_l), .VelR (vel_r),
        .CountL (cnt_l), .CountR (cnt_r),
        .CurYaw (AngYaw),
        .YawValid (AngValid),           
        .Override (Override), .OverDir (OverDir), .OverSpeed (OverSpeed),
        .LoopEn (LoopEn),
        .DirCmd (mc_dir), .SpeedPct (mc_spd),
        .EffDir (mc_eff_dir), .EffSpeed (mc_eff_spd),
        .DutyL (mc_dutyl), .DutyR (mc_dutyr),
        .ClosdLoopEn (mc_closed),
        .StMode (), .StDir (),
        .StVelL (), .StVelR (),
        .StOdomL (mc_odoml), .StOdomR (mc_odomr),
        .RotBusy (mc_rotbusy), .Estop (mc_estop), .EncNoPulse (mc_nopulse),
        .StHdgCorr (StHdgCorr), .StHdgAct (StHdgAct)
    );

    motor_drive #(
        .CLK_FREQ (CLK_FREQ)
    ) u_drive (
        .Clk (Clk), .Rst_n (Rst_n),

        .DirCmd (mc_eff_dir), .SpeedPct (mc_eff_spd),
        .ClosdLoopEn (mc_closed),
        .DutyL (mc_dutyl), .DutyR (mc_dutyr),
        .MotorPwm (MotorPwm), .MotorDir (MotorDir), .MotorDirN (MotorDirN),
        .DutyOutL (duty_out_l), .DutyOutR (duty_out_r)
    );

    generate
    if(AVOID_EN || NAV_EN || SEG1_EN) begin : g_task34

        assign us_dist_in  = {us_d2, us_d1, us_d0};
        assign us_valid_in = {us_v2, us_v1, us_v0};

        us_poll u_poll (
            .Clk (Clk), .Rst_n (Rst_n),
            .En (1'b1),
            .ValidIn (us_valid_in), .DistanceIn (us_dist_in),
            .Start (us_start),
            .DistanceOut (us_dist_out),
            .TimeoutFlag (us_timeout),
            .DoneMask (us_done),
            .RoundCnt (), .CurIdx ()
        );

        ultrasonic_hcsr04 #(.FreeRun (0)) u_us_f (
            .Clk (Clk), .Rst_n (Rst_n),
            .Start (us_start[0]), .Trig (UsTrig0), .Echo (UsEcho0),
            .Distance (us_d0), .Valid (us_v0), .Busy (), .TimeoutCnt ()
        );
        ultrasonic_hcsr04 #(.FreeRun (0)) u_us_l (
            .Clk (Clk), .Rst_n (Rst_n),
            .Start (us_start[1]), .Trig (UsTrig1), .Echo (UsEcho1),
            .Distance (us_d1), .Valid (us_v1), .Busy (), .TimeoutCnt ()
        );
        ultrasonic_hcsr04 #(.FreeRun (0)) u_us_r (
            .Clk (Clk), .Rst_n (Rst_n),
            .Start (us_start[2]), .Trig (UsTrig2), .Echo (UsEcho2),
            .Distance (us_d2), .Valid (us_v2), .Busy (), .TimeoutCnt ()
        );

        assign v_avg = ({vel_l[15], vel_l} + {vel_r[15], vel_r}) >>> 1;

        always @(posedge Clk or negedge Rst_n)
        if(!Rst_n)       dm_pr <= 48'sd0;
        else if(Tick)    dm_pr <= v_avg * `ANG_D100_DIV100_K;    

        assign dist_mm = dm_pr[37:22];

        odom_xy #(.CLK_FREQ (CLK_FREQ)) u_odom (
            .Clk (Clk), .Rst_n (Rst_n), .Tick (Tick),
            .DistMm (dist_mm), .Yaw (AngYaw),
            .CurX (cur_x), .CurY (cur_y)
        );

        avoid_fsm #(.CLK_FREQ (CLK_FREQ)) u_avoid (
            .Clk (Clk), .Rst_n (Rst_n),
            .En ((AVOID_EN != 0) & mode_avoid), .Tick (Tick),
            .RangeF (us_dist_out[15:0]),         
            .RangeRear (us_dist_out[31:16]),     
            .CmdDir (mc_dir), .CmdSpeed (mc_spd),
            .DirOut (av_dir), .SpeedOut (av_spd),
            .AvoidActive (av_active), .Near (av_near),
            .State (av_state)
        );

        if(SEG1_EN != 0) begin : g_seg1_on

            wire s1_beep_raw;       

            task4_seg1_fsm #(
                .CLK_FREQ       (CLK_FREQ)

            ) u_seg1 (
                .Clk       (Clk), .Rst_n (Rst_n), .Tick (Tick),
                .En        (mode_seg1),
                .Start     (cmd_seg1_start),
                .Abort     (cmd_seg1_abort),
                .Estop     (Estop),
                .RangeF    (us_dist_out[15:0]),      
                .DistMm    (dist_mm),                
                .CurYaw    (AngYaw),                 
                .YawValid  (AngValid),
                .CmdDir    (mc_dir), .CmdSpeed (mc_spd),
                .DirOut    (s1_dir), .SpeedOut (s1_spd),
                .Active    (s1_act), .State (s1_state), .Done (s1_done),
                .Leg2Mm    (s1_dist), .TotalMm (), .YawErrD100 (), .ErrCode (s1_err),
                .Beep      (s1_beep_raw)       
            );

            assign s1_beep = s1_beep_raw & mode_seg1;

            assign StSeg1State  = s1_state;
            assign StSeg1Done   = s1_done;
            assign StSeg1DistMm = s1_dist;
            assign StSeg1Err    = s1_err;

        end
        else begin : g_seg1_off

            assign s1_dir   = 8'h00;
            assign s1_spd   = 8'h00;
            assign s1_act   = 1'b0;
            assign s1_state = 8'h00;
            assign s1_done  = 1'b0;
            assign s1_dist  = 32'sd0;
            assign s1_err   = 8'h00;
            assign s1_beep  = 1'b0;        

            assign StSeg1State  = 8'h00;
            assign StSeg1Done   = 1'b0;
            assign StSeg1DistMm = 32'sd0;
            assign StSeg1Err    = 8'h00;

        end

        nav_fsm #(
            .CLK_FREQ  (CLK_FREQ),
            .ARRIVE_CM (NAV_ARRIVE_CM)
        ) u_nav (
            .Clk (Clk), .Rst_n (Rst_n),
            .En ((NAV_EN != 0) & mode_nav), .Tick (Tick),
            .Start (cmd_nav_start), .Abort (cmd_nav_abort), .WpNew (cmd_wp_new),
            .WpX (wp_x), .WpY (wp_y),
            .CurX (cur_x), .CurY (cur_y), .CurYaw (AngYaw),
            .RangeF (us_dist_out[15:0]),
            .State (nav_state), .WpIdx (nav_wp),
            .DirOut (nav_dir), .SpeedOut (nav_spd),
            .Active (nav_active),
            .RemainCm (StRemainCm)
        );

        assign StAvoidState  = av_state;
        assign StAvoidNear   = av_near;
        assign StAvoidActive = av_active;

    end
    else begin : g_task2only

        assign us_dist_in   = 48'd0;
        assign us_valid_in  = 3'b000;
        assign us_start     = 3'b000;
        assign us_dist_out  = {`US_INVALID, `US_INVALID, `US_INVALID};
        assign us_timeout   = 3'b000;
        assign us_done      = 3'b000;

        assign UsTrig0      = 1'b0;
        assign UsTrig1      = 1'b0;
        assign UsTrig2      = 1'b0;

        assign av_dir       = 8'h00;
        assign av_spd       = 8'h00;
        assign av_active    = 1'b0;
        assign av_near      = 1'b0;

        assign nav_dir      = 8'h00;
        assign nav_spd      = 8'h00;
        assign nav_state    = `NAV_IDLE;
        assign nav_wp       = 8'h00;
        assign nav_active   = 1'b0;

        assign cur_x        = 16'sd0;
        assign cur_y        = 16'sd0;
        assign StRemainCm   = 16'sd0;

        assign StAvoidState  = 8'h00;
        assign StAvoidNear   = 1'b0;
        assign StAvoidActive = 1'b0;

        assign s1_dir   = 8'h00;
        assign s1_spd   = 8'h00;
        assign s1_act   = 1'b0;
        assign s1_state = 8'h00;
        assign s1_done  = 1'b0;
        assign s1_dist  = 32'sd0;
        assign s1_err   = 8'h00;

        assign StSeg1State  = 8'h00;
        assign StSeg1Done   = 1'b0;
        assign StSeg1DistMm = 32'sd0;
        assign StSeg1Err    = 8'h00;

        assign dist_mm      = 16'sd0;
        assign v_avg        = 17'sd0;

    end
    endgenerate

    assign ov_nav    = (NAV_EN != 0) & mode_nav & nav_active;

    assign ov_s1     = (SEG1_EN != 0) & mode_seg1 & s1_act;
    assign Override  = ((AVOID_EN != 0) & mode_avoid) | ov_nav | ov_s1 | Estop | SafetyStop;
    assign OverDir   = (Estop | SafetyStop) ? `DIR_STOP :
                       ov_s1   ? s1_dir     :
                       ov_nav  ? nav_dir    : av_dir;
    assign OverSpeed = (Estop | SafetyStop) ? 8'd0        :
                       ov_s1   ? s1_spd     :
                       ov_nav  ? nav_spd    : av_spd;

    sync_2ff u_sync_estop (
        .Clk (Clk), .Rst_n (Rst_n),
        .AsyncIn (Key[0]), .SyncOut (key_s1_sync)
    );

    assign Estop = mc_estop | ~key_s1_sync;     

    assign StFault[`FAULT_IMU_NODATA]  = ~ImuLink;              
    assign StFault[`FAULT_US_TIMEOUT]  = |us_timeout;           
    assign StFault[`FAULT_ENC_NOPULSE] = mc_nopulse;            
    assign StFault[`FAULT_MOTOR_DRV]   = 1'b0;                  
    assign StFault[`FAULT_LINK_LOST]   = ~LinkAlive;            
    assign StFault[`FAULT_LOW_BAT]     = 1'b0;                  
    assign StFault[`FAULT_ESTOP]       = Estop;                 
    assign StFault[7]                  = 1'b0;                  

    assign StMode     = mode_r;
    assign StDir      = Override ? OverDir : mc_dir;
    assign StDutyL    = duty_out_l;
    assign StDutyR    = duty_out_r;
    assign StVelL     = vel_l;
    assign StVelR     = vel_r;
    assign StOdomL    = mc_odoml;
    assign StOdomR    = mc_odomr;
    assign StRangeF   = us_dist_out[15:0];
    assign StRangeL   = us_dist_out[31:16];
    assign StRangeR   = us_dist_out[47:32];
    assign StNavState = nav_state;
    assign StNavWp    = nav_wp;
    assign StNavX     = cur_x;
    assign StNavY     = cur_y;
    assign StNavYaw   = AngYaw;

    assign BLed = DBG_ENCDIR_EN
        ? { enc_dir_r, enc_dir_l, enc_live_r, enc_live_l }
        : { LinkAlive, nav_active | mode_avoid,
            enc_live_l | enc_live_r, av_near | Estop | SafetyStop };

    localparam [31:0] BEEP_EN_TO = CLK_FREQ * 3;

    reg        beep_en_r;
    reg [31:0] beep_to_cnt;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        beep_en_r   <= 1'b0;
        beep_to_cnt <= 32'd0;
    end
    else if(CmdValid && (CmdCode == `CMD_BEEP)) begin
        beep_en_r   <= CmdArg0[0];
        beep_to_cnt <= 32'd0;                   
    end
    else if(beep_en_r) begin
        if(beep_to_cnt >= BEEP_EN_TO - 32'd1) begin
            beep_en_r   <= 1'b0;                
            beep_to_cnt <= 32'd0;
        end
        else
            beep_to_cnt <= beep_to_cnt + 32'd1;
    end
    else
        beep_to_cnt <= 32'd0;

    wire beep_alarm = (av_near & ~mode_seg1) | Estop | s1_beep | seg1_beep_led | beep_en_r;

    localparam [15:0] BEEP_HALF = CLK_FREQ / (2 * 4000);

    reg [15:0] beep_cnt;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n)                        beep_cnt <= 16'd0;
    else if(beep_cnt >= BEEP_HALF - 16'd1) beep_cnt <= 16'd0;
    else                              beep_cnt <= beep_cnt + 16'd1;

    assign Beep = beep_alarm & (beep_cnt < (BEEP_HALF >> 1));

endmodule

