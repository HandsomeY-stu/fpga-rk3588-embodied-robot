/*
 * motion_ctrl.v
 * 编码器速度闭环、电机目标速度分配与里程反馈，支持直行和差速转向。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module motion_ctrl #(
    parameter CLK_FREQ     = `CLK_CTRL_HZ,
    parameter V_MAX_MMPS   = 500,           
    parameter ROT_TOL_D100 = 300,           
    parameter ROT_MMPS     = 250,           
    parameter NOPULSE_CNT  = 100,           

    parameter integer TRIM_L_Q10 = 1024,
    parameter integer TRIM_R_Q10 = 1024,

    parameter integer HDG_EN          = 0,      
    parameter integer HDG_KP_Q8       = 64,     

    parameter integer HDG_MAX_MMPS    = 150,    
    parameter integer HDG_TOL_D100    = 20,     
    parameter integer HDG_STALE_TICKS = 30,     

    parameter ENC_CPR      = `ENC_CPR,
    parameter GEAR_RATIO   = `GEAR_RATIO,
    parameter ENC_QUAD     = `ENC_QUAD,
    parameter WHEEL_DIAM_MM= `WHEEL_DIAM_MM
)(
    input                    Clk,
    input                    Rst_n,

    input                    Tick,         

    input                    CmdValid,
    input      [7:0]         CmdCode,
    input      [15:0]        CmdArg0,
    input      [15:0]        CmdArg1,
    input      [15:0]        CmdArg2,

    input      signed [15:0] VelL, VelR,           
    input      signed [31:0] CountL, CountR,       
    input      signed [15:0] CurYaw,               
    input                    YawValid,             

    input                    Override,             
    input      [7:0]         OverDir,
    input      [7:0]         OverSpeed,

    input                    LoopEn,               

    output reg [7:0]         DirCmd,               
    output reg [7:0]         SpeedPct,             

    output reg [7:0]         EffDir,
    output reg [7:0]         EffSpeed,
    output reg signed [15:0] DutyL, DutyR,         
    output reg               ClosdLoopEn,          

    output reg [7:0]         StMode,
    output reg [7:0]         StDir,
    output     signed [15:0] StVelL, StVelR,
    output reg signed [31:0] StOdomL, StOdomR,

    output reg               RotBusy,              
    output reg               Estop,                
    output reg               EncNoPulse,            

    output     signed [15:0] StHdgCorr,
    output                   StHdgAct              
);

    localparam integer CNT_PER_REV = ENC_CPR * GEAR_RATIO * ENC_QUAD;       
    localparam integer MM_PER_REV  = (WHEEL_DIAM_MM * 31416) / 10000;       
    localparam integer ODOM_Q20    = (MM_PER_REV << 20) / CNT_PER_REV;      

    localparam integer K_BASE = (V_MAX_MMPS << 20) / 100;

    localparam signed [15:0] K_GAIN = 16'sd1049;

    localparam signed [15:0] ROT_V = ROT_MMPS;

    reg signed [15:0] kp_q10, ki_q10, kd_q10;
    reg signed [16:0] rot_target;       

    reg [7:0]         rot_last_dir;     
    reg [15:0]        rot_last_ang;

    reg signed [31:0] kp_pr, ki_pr, kd_pr;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        kp_q10 <= 16'sd2048;    
        ki_q10 <= 16'sd512;     
        kd_q10 <= 16'sd0;       
        kp_pr  <= 32'sd0;
        ki_pr  <= 32'sd0;
        kd_pr  <= 32'sd0;
    end
    else begin
        kp_pr <= $signed(CmdArg0) * K_GAIN;
        ki_pr <= $signed(CmdArg1) * K_GAIN;
        kd_pr <= $signed(CmdArg2) * K_GAIN;

        if(CmdValid && (CmdCode == `CMD_PID_SET)) begin
            kp_q10 <= kp_pr[25:10];
            ki_q10 <= ki_pr[25:10];
            kd_q10 <= kd_pr[25:10];
        end
    end

    wire cmd_rot   = CmdValid && (CmdCode == `CMD_ROTATE);
    wire cmd_other = CmdValid && (CmdCode != `CMD_ROTATE);

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        DirCmd    <= `DIR_STOP;
        SpeedPct  <= 8'd0;
        StMode    <= `MODE_MANUAL;
        Estop     <= 1'b0;
        rot_target<= 17'sd0;
        rot_last_dir <= 8'hFF;
        rot_last_ang <= 16'hFFFF;
    end
    else if(CmdValid) begin

        if(CmdCode != `CMD_ROTATE) rot_last_dir <= 8'hFF;

        case(CmdCode)
        `CMD_MOTION: begin
            DirCmd   <= CmdArg0[7:0];
            SpeedPct <= CmdArg0[15:8];
            Estop    <= 1'b0;
        end
        `CMD_SPEED: begin
            SpeedPct <= CmdArg0[7:0];
            Estop    <= 1'b0;
        end
        `CMD_MODE: begin
            StMode <= CmdArg0[7:0];
        end
        `CMD_STOP: begin                      
            DirCmd   <= `DIR_STOP;
            SpeedPct <= 8'd0;
            Estop    <= 1'b1;
        end
        `CMD_ROTATE: begin

            if(!RotBusy && ((CmdArg0[7:0] != rot_last_dir) || (CmdArg1 != rot_last_ang))) begin
                rot_target   <= (CmdArg0[7:0] == `DIR_SPIN_L)
                                ? ({CurYaw[15], CurYaw} + $signed({CmdArg1[15], CmdArg1}))
                                : ({CurYaw[15], CurYaw} - $signed({CmdArg1[15], CmdArg1}));
                rot_last_dir <= CmdArg0[7:0];
                rot_last_ang <= CmdArg1;
            end
            SpeedPct   <= 8'd50;              
            Estop      <= 1'b0;
        end
        default: ;
        endcase
    end

    wire signed [17:0] rot_err_raw = {rot_target[16], rot_target} - {{2{CurYaw[15]}}, CurYaw};

    reg signed [17:0] rot_err;
    always @(*) begin
        if(rot_err_raw > 18'sd18000)       rot_err = rot_err_raw - 18'sd36000;
        else if(rot_err_raw < -18'sd18000) rot_err = rot_err_raw + 18'sd36000;
        else                               rot_err = rot_err_raw;
    end

    wire signed [17:0] rot_err_abs = rot_err[17] ? (~rot_err + 18'sd1) : rot_err;

    localparam integer ROT_TIMEOUT_TICKS = 800;    

    reg [15:0] rot_to_cnt;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n)                        rot_to_cnt <= 16'd0;
    else if(!RotBusy)                 rot_to_cnt <= 16'd0;
    else if(Tick && rot_to_cnt != 16'hFFFF) rot_to_cnt <= rot_to_cnt + 16'd1;

    wire rot_timeout = RotBusy && (rot_to_cnt >= ROT_TIMEOUT_TICKS);

    wire rot_accept = !RotBusy && ((CmdArg0[7:0] != rot_last_dir) || (CmdArg1 != rot_last_ang));

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n)                                        RotBusy <= 1'b0;
    else if(cmd_other)                                RotBusy <= 1'b0;
    else if(cmd_rot && rot_accept)                    RotBusy <= 1'b1;
    else if(RotBusy && (rot_err_abs <= ROT_TOL_D100)) RotBusy <= 1'b0;
    else if(rot_timeout)                              RotBusy <= 1'b0;

    wire [7:0] eff_dir   = RotBusy ? (rot_err[17] ? `DIR_SPIN_R : `DIR_SPIN_L) :
                           Override ? OverDir : DirCmd;
    wire [7:0] eff_speed = RotBusy ? SpeedPct :
                           Override ? OverSpeed : SpeedPct;

    reg [47:0] base_pr;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) base_pr <= 48'd0;
    else       base_pr <= eff_speed * K_BASE;

    wire signed [15:0] base_v = base_pr[35:20];       

    wire signed [31:0] base_l_pr = ($signed({{16{base_v[15]}}, base_v}) * TRIM_L_Q10) >>> 10;
    wire signed [31:0] base_r_pr = ($signed({{16{base_v[15]}}, base_v}) * TRIM_R_Q10) >>> 10;
    wire signed [15:0] base_l = base_l_pr[15:0];
    wire signed [15:0] base_r = base_r_pr[15:0];

    localparam signed [31:0] HDG_KP_S  = HDG_KP_Q8;
    localparam signed [31:0] HDG_MAX_S = HDG_MAX_MMPS;
    localparam signed [16:0] HDG_TOL_S = HDG_TOL_D100;

    wire hdg_run = ((eff_dir == `DIR_FWD) || (eff_dir == `DIR_BACK)) && (base_v != 16'sd0);

    reg               hdg_run_d;       
    reg               hdg_set;         
    reg signed [15:0] hdg_tgt;         
    reg [15:0]        yaw_age;         

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        hdg_run_d <= 1'b0;
        hdg_set   <= 1'b0;
        hdg_tgt   <= 16'sd0;
        yaw_age   <= 16'hFFFF;         
    end
    else begin
        hdg_run_d <= hdg_run;

        if(YawValid)                              yaw_age <= 16'd0;
        else if(Tick && (yaw_age != 16'hFFFF))    yaw_age <= yaw_age + 16'd1;

        if(!hdg_run)                              hdg_set <= 1'b0;
        else if(!hdg_run_d && (yaw_age <= HDG_STALE_TICKS)) begin
            hdg_tgt <= CurYaw;
            hdg_set <= 1'b1;
        end
    end

    wire yaw_fresh = (yaw_age <= HDG_STALE_TICKS);

    wire signed [16:0] hdg_d = {hdg_tgt[15], hdg_tgt} - {CurYaw[15], CurYaw};

    reg signed [16:0] hdg_err;
    always @(*) begin
        if(hdg_d > 17'sd18000)       hdg_err = hdg_d - 17'sd36000;
        else if(hdg_d < -17'sd18000) hdg_err = hdg_d + 17'sd36000;
        else                         hdg_err = hdg_d;
    end

    wire signed [16:0] hdg_e = (hdg_err >  HDG_TOL_S) ? (hdg_err - HDG_TOL_S) :
                               (hdg_err < -HDG_TOL_S) ? (hdg_err + HDG_TOL_S) : 17'sd0;

    wire signed [31:0] hdg_pr = ($signed(hdg_e) * HDG_KP_S) >>> 8;

    wire signed [31:0] hdg_cl = (hdg_pr >  HDG_MAX_S) ?  HDG_MAX_S :
                                (hdg_pr < -HDG_MAX_S) ? -HDG_MAX_S : hdg_pr;

    wire signed [15:0] hdg_corr = hdg_cl[15:0];

    wire hdg_act = (HDG_EN != 0) && hdg_run && hdg_set && yaw_fresh;

    assign StHdgCorr = hdg_act ? hdg_corr : 16'sd0;
    assign StHdgAct  = hdg_act;

    reg signed [15:0] tgt_l, tgt_r;

    always @(*) begin
        if(RotBusy) begin

            tgt_l = rot_err[17] ?  ROT_V : -ROT_V;
            tgt_r = rot_err[17] ? -ROT_V :  ROT_V;
        end
        else case(eff_dir)
        `DIR_FWD:    begin

            tgt_l = hdg_act ? (base_l - hdg_corr) : base_l;
            tgt_r = hdg_act ? (base_r + hdg_corr) : base_r;
        end
        `DIR_BACK:   begin

            tgt_l = hdg_act ? (-base_l - hdg_corr) : -base_l;
            tgt_r = hdg_act ? (-base_r + hdg_corr) : -base_r;
        end
        // 行进差速：内侧轮为外侧一半；提高该档速度也会提高角速度。
        `DIR_LEFT:   begin tgt_l =  (base_l >>> 1); tgt_r =  base_r;         end
        `DIR_RIGHT:  begin tgt_l =  base_l;         tgt_r =  (base_r >>> 1); end
        `DIR_SPIN_L: begin tgt_l = -base_l;         tgt_r =  base_r;         end
        `DIR_SPIN_R: begin tgt_l =  base_l;         tgt_r = -base_r;         end
        default:     begin tgt_l = 16'sd0;          tgt_r = 16'sd0;          end
        endcase
    end

    reg [15:0] nopulse_l, nopulse_r;
    reg        enc_np_l,  enc_np_r;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        nopulse_l <= 16'd0; nopulse_r <= 16'd0;
        enc_np_l  <= 1'b0;  enc_np_r  <= 1'b0;
    end
    else if(Tick) begin

        if((tgt_l != 16'sd0) && (VelL == 16'sd0)) begin
            if(nopulse_l >= NOPULSE_CNT - 1) enc_np_l <= 1'b1;
            else                             nopulse_l <= nopulse_l + 1'b1;
        end
        else begin
            nopulse_l <= 16'd0;
            enc_np_l  <= 1'b0;
        end

        if((tgt_r != 16'sd0) && (VelR == 16'sd0)) begin
            if(nopulse_r >= NOPULSE_CNT - 1) enc_np_r <= 1'b1;
            else                             nopulse_r <= nopulse_r + 1'b1;
        end
        else begin
            nopulse_r <= 16'd0;
            enc_np_r  <= 1'b0;
        end
    end

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) EncNoPulse <= 1'b0;
    else       EncNoPulse <= enc_np_l | enc_np_r;

    wire motion_active = (tgt_l != 16'sd0) || (tgt_r != 16'sd0);

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) ClosdLoopEn <= 1'b0;
    else       ClosdLoopEn <= LoopEn & motion_active & ~EncNoPulse & ~Estop;

    wire signed [15:0] pid_l, pid_r;

    pid_ctrl u_pid_l (
        .Clk (Clk), .Rst_n (Rst_n),
        .Enable (ClosdLoopEn), .Clr (1'b0), .Tick (Tick),
        .KpQ10 (kp_q10), .KiQ10 (ki_q10), .KdQ10 (kd_q10),
        .Setpoint (tgt_l), .Feedback (VelL),
        .Out (pid_l), .ErrOut (), .Sat ()
    );

    pid_ctrl u_pid_r (
        .Clk (Clk), .Rst_n (Rst_n),
        .Enable (ClosdLoopEn), .Clr (1'b0), .Tick (Tick),
        .KpQ10 (kp_q10), .KiQ10 (ki_q10), .KdQ10 (kd_q10),
        .Setpoint (tgt_r), .Feedback (VelR),
        .Out (pid_r), .ErrOut (), .Sat ()
    );

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        DutyL <= 16'sd0;
        DutyR <= 16'sd0;
        StDir <= `DIR_STOP;
        EffDir   <= `DIR_STOP;
        EffSpeed <= 8'd0;
    end
    else begin
        DutyL <= pid_l;
        DutyR <= pid_r;
        StDir <= eff_dir;
        EffDir   <= eff_dir;
        EffSpeed <= eff_speed;
    end

    assign StVelL = VelL;
    assign StVelR = VelR;

    reg signed [31:0] cnt_prev_l, cnt_prev_r;
    reg signed [47:0] dl_pr, dr_pr;

    wire signed [23:0] dl_q10 = dl_pr[33:10];
    wire signed [23:0] dr_q10 = dr_pr[33:10];

    reg signed [31:0] odom_l_q10, odom_r_q10;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        cnt_prev_l <= 32'sd0;
        cnt_prev_r <= 32'sd0;
        dl_pr      <= 48'sd0;
        dr_pr      <= 48'sd0;
        odom_l_q10 <= 32'sd0;
        odom_r_q10 <= 32'sd0;
        StOdomL    <= 32'sd0;
        StOdomR    <= 32'sd0;
    end
    else if(Tick) begin
        cnt_prev_l <= CountL;
        cnt_prev_r <= CountR;
        dl_pr      <= (CountL - cnt_prev_l) * ODOM_Q20;
        dr_pr      <= (CountR - cnt_prev_r) * ODOM_Q20;
        odom_l_q10 <= odom_l_q10 + dl_q10;
        odom_r_q10 <= odom_r_q10 + dr_q10;

        StOdomL    <= (odom_l_q10 + 32'sd512) >>> 10;
        StOdomR    <= (odom_r_q10 + 32'sd512) >>> 10;
    end

endmodule
