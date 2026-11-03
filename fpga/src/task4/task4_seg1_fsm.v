/*
 * task4_seg1_fsm.v
 * 任务四固定路线状态机：双障碍绕行、累计里程和v48尾段三次左转。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module task4_seg1_fsm #(
    parameter CLK_FREQ = `CLK_CTRL_HZ,

    parameter [7:0] CRUISE_SPEED      = 8'd50,    
    parameter [7:0] SLOW_SPEED        = 8'd50,    

    parameter [7:0] ARC_SPEED         = 8'd50,    
    parameter [7:0] TURN_SPEED_COARSE = 8'd6,
    parameter [7:0] TURN_SPEED_FINE   = 8'd6,     

    parameter [7:0] RAMP_STEP         = 8'd5,     

    parameter [15:0] SLOW_MM     = `AVOID_SLOW_MM,     
    parameter [15:0] TURN_AT_MM  = `AVOID_STOP_MM,     

    parameter integer YAW_INV_FIXED  = 1,
    parameter [15:0] SWERVE_AT_MM    = 16'd250,    

    parameter [15:0] SWERVE_AT_MM2   = 16'd320,    
    parameter [15:0] SW2_BLIND_MM    = 16'd150,    
    parameter [15:0] SWERVE_ANG_D100 = 16'd7500,   

    parameter integer RET_ADJ_TICKS  = 8,     

    parameter integer RET_ADJ_TICKS2 = 8,
    parameter [15:0] PASS_MM         = 16'd0,      
    parameter [15:0] BLIND_MM        = 16'd450,    
    parameter [15:0] CLEAR_MM    = `AVOID_CLEAR_MM,    
    parameter [15:0] GUARD_MM    = 16'd120,            
    parameter [15:0] D2_MM       = 16'd300,            
    parameter [15:0] D3_MM       = 16'd500,            
    parameter [15:0] D4_MM       = 16'd300,            
    parameter [15:0] D5_MM       = 16'd750,            

    parameter [15:0] D6_MM       = 16'd600,            
    parameter [15:0] D7_MM       = 16'd1000,           
    parameter [15:0] D8_MM       = 16'd500,            

    parameter integer TAIL_ALWAYS_TURN = 1,

    parameter integer TAIL_TRIM_EN    = 1,
    parameter integer TAIL_TRIM_DB    = 80,     
    parameter integer TAIL_TRIM_MAX   = 3,      
    parameter integer TAIL_TRIM_PASS  = 12,     
    parameter [15:0] LEAD_MM     = 16'd8,              
    parameter [15:0] LEG1_MAX_MM = 16'd3000,           

    parameter signed [15:0] TURN_ANGLE_D100 = 16'sd9000,   
    parameter signed [15:0] TOL_D100        = 16'sd300,    
    parameter signed [15:0] FINE_BAND_D100  = 16'sd9000,   
    parameter signed [15:0] ACCEPT_D100     = 16'sd600,    
    parameter [7:0]         FLIP_MAX        = 8'd8,        

    parameter integer PARK_MS         = 300,     
    parameter [7:0] CAL_SPEED         = 8'd15,   

    parameter integer CAL_MS          = 600,     
    parameter signed [15:0] CAL_MIN_D100 = 16'sd300,   
    parameter integer SETTLE_MS       = 100,      
    parameter integer BRAKE_MS        = 200,     
    parameter integer TURN_TIMEOUT_MS = 8000,     
    parameter integer SEG_TIMEOUT_MS  = 70000,    
    parameter integer IMU_STALE_TICKS = 30        
)(
    input                    Clk,
    input                    Rst_n,

    input                    En,         
    input                    Start,      
    input                    Abort,      
    input                    Estop,      
    input                    Tick,       

    input      [15:0]        RangeF,     
    input      signed [15:0] DistMm,     
    input      signed [15:0] CurYaw,     
    input                    YawValid,   

    input      [7:0]         CmdDir,     
    input      [7:0]         CmdSpeed,   

    output reg [7:0]         DirOut,     
    output reg [7:0]         SpeedOut,   
    output reg               Active,     
    output reg [7:0]         State,      
    output reg               Done,       

    output reg signed [31:0] Leg2Mm,     
    output reg signed [31:0] TotalMm,    
    output reg signed [15:0] YawErrD100, 
    output reg [7:0]         ErrCode,    

    output                   Beep

);

    localparam [7:0] S_IDLE   = 8'd0,    
                     S_CRUISE = 8'd1,    
                     S_SLOW   = 8'd2,    
                     S_STOP   = 8'd3,    
                     S_TURN   = 8'd4,    
                     S_ALIGN  = 8'd5,    
                     S_RUN2   = 8'd6,    
                     S_BRAKE  = 8'd7,    
                     S_DONE   = 8'd8,    
                     S_ABORT  = 8'd9,    
                     S_CAL    = 8'd10,   
                     S_RUN3   = 8'd11,   

                     S_RUN4   = 8'd12,   
                     S_RUN5   = 8'd13,   

                     S_RUN6   = 8'd14,   
                     S_RUN7   = 8'd15,   
                     S_RUN8   = 8'd16,   
                     S_TAILTRIM = 8'd17;  

    localparam integer PARK_CNT    = (PARK_MS         / `CTRL_PERIOD_MS);
    localparam integer CAL_CNT     = (CAL_MS          / `CTRL_PERIOD_MS);
    localparam integer SETTLE_CNT  = (SETTLE_MS       / `CTRL_PERIOD_MS);
    localparam integer BRAKE_CNT   = (BRAKE_MS        / `CTRL_PERIOD_MS);
    localparam integer TURN_CNT    = (TURN_TIMEOUT_MS / `CTRL_PERIOD_MS);
    localparam integer SEG_CNT     = (SEG_TIMEOUT_MS  / `CTRL_PERIOD_MS);

    localparam signed [17:0] TURN_ANG_S = TURN_ANGLE_D100;
    localparam signed [17:0] TOL_S      = TOL_D100;
    localparam signed [17:0] FINE_S     = FINE_BAND_D100;
    localparam signed [17:0] ACCEPT_S   = ACCEPT_D100;
    localparam signed [17:0] CALMIN_S   = CAL_MIN_D100;

    function signed [17:0] wrap18;
        input signed [17:0] a;
        begin
            if(a > 18'sd18000)       wrap18 = a - 18'sd36000;
            else if(a < -18'sd18000) wrap18 = a + 18'sd36000;
            else                     wrap18 = a;
        end
    endfunction

    function [7:0] dir_for;
        input signed [17:0] tgt;
        input signed [17:0] cur;
        reg   signed [17:0] d;
        begin
            d = wrap18(tgt - cur);
            dir_for = (d > 18'sd0) ? `DIR_SPIN_L : `DIR_SPIN_R;
        end
    endfunction
    localparam signed [31:0] D2_S       = D2_MM;
    localparam signed [31:0] D3_S       = D3_MM;
    localparam signed [31:0] D4_S       = D4_MM;
    localparam signed [31:0] D5_S       = D5_MM;

    localparam signed [31:0] D6_S       = D6_MM;
    localparam signed [31:0] D7_S       = D7_MM;
    localparam signed [31:0] D8_S       = D8_MM;

    localparam signed [17:0] TURN_2S    = TURN_ANG_S + TURN_ANG_S;
    localparam signed [17:0] TURN_3S    = TURN_2S    + TURN_ANG_S;
    localparam signed [17:0] SWERVE_ANG_S = SWERVE_ANG_D100;
    localparam signed [31:0] SWERVE_AT_S  = SWERVE_AT_MM;
    localparam signed [31:0] SWERVE_AT2_S = SWERVE_AT_MM2;
    localparam signed [31:0] SW2BLIND_S   = SW2_BLIND_MM;
    localparam signed [31:0] BLIND_S      = BLIND_MM;
    localparam signed [31:0] PASS_S       = PASS_MM;
    localparam signed [31:0] LEAD_S     = LEAD_MM;
    localparam signed [31:0] LEG1MAX_S  = LEG1_MAX_MM;

    localparam [15:0] SLOW_S   = SLOW_MM;
    localparam [15:0] TURNAT_S = TURN_AT_MM;
    localparam [15:0] CLEAR_S  = CLEAR_MM;
    localparam [15:0] GUARD_S  = GUARD_MM;

    reg               obs_cnt;            
    reg  [1:0]        turn_mode;          
    reg  signed [31:0] blind_mm;          
    reg               cal_ready;          
    reg  [7:0]        out_dir;
    reg  [1:0]        trim_cnt;           
    reg  [15:0]       arc_ticks;          
    reg               not_aligned;        
    wire f_valid = (RangeF != `US_INVALID);
    wire f_stop  = f_valid && (RangeF <= TURNAT_S);
    wire f_guard = f_valid && (RangeF <= GUARD_S) && (blind_mm <= 32'sd0);    
    wire f_slow  = f_valid && (RangeF <= SLOW_S);

    wire [15:0] sw_at_now = (obs_cnt == 1'b1) ? SWERVE_AT2_S[15:0] : SWERVE_AT_S[15:0];
    wire        sw_blind_ok = (blind_mm <= 32'sd0) ||
                              ((obs_cnt == 1'b1) && (blind_mm <= SW2BLIND_S));
    wire f_sw    = f_valid && (RangeF <= sw_at_now) && sw_blind_ok;   
    wire f_clear = (!f_valid) || (RangeF >= CLEAR_S);

    reg [15:0] yaw_age;       
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n)                        yaw_age <= 16'hFFFF;
    else if(YawValid)                 yaw_age <= 16'd0;
    else if(Tick && (yaw_age != 16'hFFFF)) yaw_age <= yaw_age + 16'd1;

    wire yaw_fresh = (yaw_age <= IMU_STALE_TICKS);

    reg yaw_new;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n)      yaw_new <= 1'b0;
    else if(YawValid) yaw_new <= 1'b1;
    else if(Tick)     yaw_new <= 1'b0;

    reg  signed [17:0] turn_tgt;          
    wire signed [17:0] cur18 = {{2{CurYaw[15]}}, CurYaw};

    reg  yaw_inv;
    reg  signed [17:0] yaw_cal0;          

    reg  signed [17:0] yaw_home;          

    reg        [7:0]   spin_lat;          
    reg               leg2nd;             

    reg               phase2;             
    reg  [2:0]        leg_sel;            
    reg  signed [31:0] Leg4Mm, Leg5Mm;    
    reg  signed [31:0] Leg6Mm, Leg7Mm, Leg8Mm;   

    reg  [3:0]        trim_ticks;         
    reg  [7:0]        trim_dir;           
    reg               trim_go;            
    reg  [3:0]        pass_cnt;           
    reg  signed [31:0] Leg3Mm;            
    wire signed [17:0] yaw_ccw = yaw_inv ? (~cur18 + 18'sd1) : cur18;

    reg  signed [17:0] yaw_prev;
    reg  signed [17:0] dyaw_abs;
    wire signed [17:0] yaw_step = cur18 - yaw_prev;    

    reg  signed [17:0] yaw_step_w;
    always @(*) begin
        if(yaw_step > 18'sd18000)       yaw_step_w = yaw_step - 18'sd36000;
        else if(yaw_step < -18'sd18000) yaw_step_w = yaw_step + 18'sd36000;
        else                            yaw_step_w = yaw_step;
    end

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        yaw_prev <= 18'sd0;
        dyaw_abs <= 18'sd0;
    end
    else if(YawValid) begin
        yaw_prev <= cur18;                                      
        dyaw_abs <= yaw_step_w[17] ? (~yaw_step_w + 18'sd1) : yaw_step_w;
    end

    wire signed [17:0] stop_th     = TOL_S    + dyaw_abs;
    wire signed [17:0] stop_th_acc = ACCEPT_S + dyaw_abs;

    reg        fine_on;                   
    reg [7:0]  spin_rev_cnt;              
    reg [7:0]  prev_spin;

    wire signed [17:0] cal_d    = cur18 - yaw_cal0;
    reg  signed [17:0] cal_dw;
    always @(*) begin
        if(cal_d > 18'sd18000)       cal_dw = cal_d - 18'sd36000;
        else if(cal_d < -18'sd18000) cal_dw = cal_d + 18'sd36000;
        else                         cal_dw = cal_d;
    end
    wire               cal_flip = (cal_dw < -CALMIN_S);                         
    wire               cal_seen = (cal_dw > CALMIN_S) || (cal_dw < -CALMIN_S);  
    wire               yaw_inv_nxt = cal_seen ? cal_flip : yaw_inv;             

    // 使用目标与当前角度的差值，再规约到±180°，避免直接比较绝对航向在边界处失真。
    wire signed [17:0] yaw_d = turn_tgt - yaw_ccw;    

    reg  signed [17:0] yaw_err;
    always @(*) begin                               
        if(yaw_d > 18'sd18000)       yaw_err = yaw_d - 18'sd36000;
        else if(yaw_d < -18'sd18000) yaw_err = yaw_d + 18'sd36000;
        else                         yaw_err = yaw_d;
    end

    wire signed [17:0] err_abs = yaw_err[17] ? (~yaw_err + 18'sd1) : yaw_err;

    localparam signed [31:0] TRIM_DB_S  = TAIL_TRIM_DB;
    localparam integer       TRIM_MAX   = TAIL_TRIM_MAX;

    wire [3:0] trim_n;
    // 默认限幅1～3时，乘法结果只在205和308处变档；阈值实现节省DSP且穷举等价。
    generate if (TRIM_MAX == 3) begin : g_trim_thresholds
        assign trim_n = ($signed(err_abs) < 18'sd205) ? 4'd1 :
                        ($signed(err_abs) < 18'sd308) ? 4'd2 : 4'd3;
    end else begin : g_trim_parameter_compat
        wire signed [31:0] trim_calc = ({{14{err_abs[17]}}, err_abs} * 32'sd10) >>> 10;
        assign trim_n = (trim_calc < 32'sd1) ? 4'd1 :
                        (trim_calc > TRIM_MAX) ? 4'd3 : trim_calc[3:0];
    end endgenerate

    wire signed [17:0] lead_ang = (dyaw_abs > 18'sd80) ? 18'sd80 : dyaw_abs;    

    wire turn_crossed = (spin_lat == `DIR_SPIN_L) ? (yaw_err <=  lead_ang)
                                                 : (yaw_err >= -lead_ang);

    // v48尾段按绝对误差判停；前两段保留原判据，减少对既有绕障行为的影响。
    wire tail_crossed = (err_abs <= lead_ang);
    wire turn_done    = (turn_mode == 2'd3) ? tail_crossed : turn_crossed;

    wire [7:0] spin_dir = yaw_err[17] ? `DIR_SPIN_R : `DIR_SPIN_L;

    wire signed [15:0] yaw_err_sat = (yaw_err > 18'sd32767)  ? 16'sd32767 :
                                     (yaw_err < -18'sd32768) ? -16'sd32768 : yaw_err[15:0];

    reg [7:0] spd_r;                      
    wire [8:0] spd_next = {1'b0, spd_r} + {1'b0, RAMP_STEP};
    localparam [8:0] CRUISE_W = CRUISE_SPEED;
    localparam [8:0] SLOW_W   = SLOW_SPEED;

    assign Beep = f_valid && (RangeF <= SLOW_S);

    wire        fwd_act  = (State == S_CRUISE) || (State == S_SLOW)  ||
                           (State == S_RUN2)   || (State == S_RUN3)  ||
                           (State == S_RUN4)   || (State == S_RUN5)  ||
                           (State == S_RUN6)   || (State == S_RUN7)  ||
                           (State == S_RUN8)   || (State == S_TAILTRIM) ||
                           (State == S_TURN);
    wire        fwd_move = fwd_act && (DistMm > 16'sd0);

    wire signed [31:0] dist_raw = {{16{DistMm[15]}}, DistMm};
    wire signed [31:0] dist_ext = (dist_raw > 32'sd50) ? 32'sd0 : dist_raw;

    reg start_lat, abort_lat;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        start_lat <= 1'b0;
        abort_lat <= 1'b0;
    end
    else if(Estop) begin
        start_lat <= 1'b0;
        abort_lat <= 1'b0;
    end
    else begin

        if(Start) start_lat <= 1'b1;
        else if(Tick && ((State == S_IDLE) || (State == S_DONE) || (State == S_ABORT)))
                 start_lat <= 1'b0;

        if(Abort) abort_lat <= 1'b1;
        else if(Tick) abort_lat <= 1'b0;
    end

    reg [15:0] hold_cnt;      
    reg [15:0] settle_cnt;    
    reg [15:0] seg_cnt;       

    localparam signed [31:0] RETA1_S = RET_ADJ_TICKS;
    localparam signed [31:0] RETA2_S = RET_ADJ_TICKS2;
    wire signed [31:0] ret_adj_sel   = (obs_cnt == 1'b0) ? RETA1_S : RETA2_S;
    wire signed [31:0] arc_ticks_cal = $signed({16'd0, hold_cnt}) + ret_adj_sel;
    wire        [15:0] arc_ticks_nxt = (arc_ticks_cal < 32'sd1) ? 16'd1
                                                                 : arc_ticks_cal[15:0];

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        State      <= S_IDLE;
        DirOut     <= `DIR_STOP;
        SpeedOut   <= 8'd0;
        Active     <= 1'b0;
        Done       <= 1'b0;
        ErrCode    <= 8'd0;
        hold_cnt   <= 16'd0;
        settle_cnt <= 16'd0;
        seg_cnt    <= 16'd0;
        spd_r      <= 8'd0;
        turn_tgt   <= 18'sd0;
        Leg2Mm     <= 32'sd0;
        TotalMm    <= 32'sd0;
        YawErrD100 <= 16'sd0;
        yaw_inv    <= 1'b0;
        yaw_cal0   <= 18'sd0;
        fine_on    <= 1'b0;
        spin_rev_cnt <= 8'd0;
        prev_spin  <= `DIR_SPIN_L;
        yaw_home   <= 18'sd0;
        spin_lat   <= `DIR_SPIN_L;
        leg2nd     <= 1'b0;
        Leg3Mm     <= 32'sd0;
        phase2     <= 1'b0;
        leg_sel    <= 3'd0;
        Leg4Mm     <= 32'sd0;
        Leg5Mm     <= 32'sd0;
        Leg6Mm     <= 32'sd0;
        Leg7Mm     <= 32'sd0;
        Leg8Mm     <= 32'sd0;
        trim_go    <= 1'b0;
        trim_ticks <= 4'd0;
        trim_dir   <= `DIR_SPIN_L;
        pass_cnt   <= 4'd0;

    end

    else if(Estop || (abort_lat && (State != S_ABORT))) begin
        DirOut   <= `DIR_STOP;
        SpeedOut <= 8'd0;
        Active   <= 1'b1;
        State    <= S_ABORT;
        ErrCode  <= Estop ? 8'd5 : (ErrCode == 8'd0 ? 8'd5 : ErrCode);
        hold_cnt <= 16'd0;
        spd_r    <= 8'd0;
    end

    else if(!En) begin
        State      <= S_IDLE;
        DirOut     <= CmdDir;
        SpeedOut   <= CmdSpeed;
        Active     <= 1'b0;
        Done       <= 1'b0;
        ErrCode    <= 8'd0;
        hold_cnt   <= 16'd0;
        settle_cnt <= 16'd0;
        seg_cnt    <= 16'd0;
        spd_r      <= 8'd0;
        Leg2Mm     <= 32'sd0;
        TotalMm    <= 32'sd0;
    end

    else if(Tick) begin

        Active   <= 1'b1;
        DirOut   <= `DIR_STOP;
        SpeedOut <= 8'd0;

        if((State == S_TURN) || (State == S_ALIGN)) begin
            if(spin_dir != prev_spin) begin
                prev_spin <= spin_dir;
                if(spin_rev_cnt != 8'hFF) spin_rev_cnt <= spin_rev_cnt + 8'd1;
            end
        end
        else begin
            spin_rev_cnt <= 8'd0;
            prev_spin    <= spin_dir;
        end

        YawErrD100 <= yaw_err_sat;

        if((State != S_IDLE) && (State != S_DONE) && (State != S_ABORT)) begin
            if(seg_cnt != 16'hFFFF) seg_cnt <= seg_cnt + 16'd1;
        end
        else seg_cnt <= 16'd0;

        case(State)

        S_IDLE: begin
            DirOut     <= CmdDir;
            SpeedOut   <= CmdSpeed;
            Active     <= 1'b0;
            Done       <= 1'b0;
            ErrCode    <= 8'd0;
            hold_cnt   <= 16'd0;
            settle_cnt <= 16'd0;
            spd_r      <= 8'd0;
            Leg2Mm     <= 32'sd0;
            TotalMm    <= 32'sd0;

            if(start_lat) begin
                Active   <= 1'b1;
                DirOut   <= `DIR_FWD;     
                SpeedOut <= 8'd0;         
                spd_r    <= 8'd0;
                yaw_inv  <= 1'b0;         
                leg2nd   <= 1'b0;
                Leg3Mm   <= 32'sd0;
                phase2   <= 1'b0;
                leg_sel  <= 3'd0;
                Leg4Mm   <= 32'sd0;
                Leg5Mm   <= 32'sd0;
                Leg6Mm   <= 32'sd0;
                Leg7Mm   <= 32'sd0;
                Leg8Mm   <= 32'sd0;
                trim_go  <= 1'b0;
                obs_cnt  <= 1'b0;
                blind_mm <= 32'sd0;
                turn_mode<= 2'd0;
                cal_ready<= 1'b0;          
                State    <= S_CAL;         
            end
        end

        S_CRUISE: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_FWD;

            if(spd_next >= CRUISE_W) begin
                spd_r    <= CRUISE_SPEED;
                SpeedOut <= CRUISE_SPEED;
            end
            else begin
                spd_r    <= spd_next[7:0];
                SpeedOut <= spd_next[7:0];
            end

            if(fwd_move) begin
                TotalMm <= TotalMm + dist_ext;
                if(blind_mm > 32'sd0) blind_mm <= blind_mm - dist_ext;
            end

            if(f_guard) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd6;         
                DirOut   <= `DIR_STOP;    
                SpeedOut <= 8'd0;         
            end
            else if(f_sw) begin

                turn_mode  <= 2'd1;
                turn_tgt   <= wrap18(yaw_home + (obs_cnt ? -SWERVE_ANG_S : SWERVE_ANG_S));
                spin_lat   <= dir_for(wrap18(yaw_home + (obs_cnt ? -SWERVE_ANG_S : SWERVE_ANG_S)), yaw_ccw);
                out_dir    <= dir_for(wrap18(yaw_home + (obs_cnt ? -SWERVE_ANG_S : SWERVE_ANG_S)), yaw_ccw);
                hold_cnt   <= 16'd0;
                settle_cnt <= 16'd0;
                trim_cnt   <= 2'd0;
                not_aligned<= 1'b0;
                DirOut     <= `DIR_FWD;
                SpeedOut   <= ARC_SPEED;
                State      <= S_TURN;
            end

            else if(f_slow) begin
                State <= S_SLOW;
                spd_r <= SLOW_SPEED;
            end
            else if(TotalMm >= LEG1MAX_S) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd4;          
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
        end

        S_SLOW: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_FWD;
            SpeedOut <= SLOW_SPEED;
            spd_r    <= SLOW_SPEED;

            if(fwd_move) begin
                TotalMm <= TotalMm + dist_ext;
                if(blind_mm > 32'sd0) blind_mm <= blind_mm - dist_ext;
            end

            if(f_guard) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd6;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else if(f_sw) begin

                turn_mode  <= 2'd1;
                turn_tgt   <= wrap18(yaw_home + (obs_cnt ? -SWERVE_ANG_S : SWERVE_ANG_S));
                spin_lat   <= dir_for(wrap18(yaw_home + (obs_cnt ? -SWERVE_ANG_S : SWERVE_ANG_S)), yaw_ccw);
                hold_cnt   <= 16'd0;
                settle_cnt <= 16'd0;
                DirOut     <= `DIR_FWD;
                SpeedOut   <= ARC_SPEED;
                State      <= S_TURN;
            end

            else if(f_clear) begin
                State <= S_CRUISE;        
            end
        end

        S_STOP: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_STOP;
            SpeedOut <= 8'd0;
            spd_r    <= 8'd0;

            if(hold_cnt >= PARK_CNT - 1) begin
                if(yaw_fresh) begin
                    hold_cnt   <= 16'd0;
                    settle_cnt <= 16'd0;
                    if(phase2) begin

                        leg_sel  <= 3'd2;
                        turn_tgt <= wrap18(yaw_home - TURN_ANG_S);    

                        spin_lat <= dir_for(wrap18(yaw_home - TURN_ANG_S), yaw_ccw);

                        State    <= S_TURN;
                    end
                    else begin
                        yaw_cal0 <= cur18;       
                        State    <= S_CAL;
                    end
                end
                else begin
                    State   <= S_ABORT;
                    ErrCode <= 8'd1;                     
                end
            end
            else hold_cnt <= hold_cnt + 16'd1;
        end

        S_CAL: begin
            Active   <= 1'b1;

            if(!cal_ready) begin
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
                spd_r    <= 8'd0;
                if(yaw_fresh) begin

                    cal_ready <= 1'b1;
                    yaw_inv   <= (YAW_INV_FIXED != 0);
                    yaw_home  <= wrap18((YAW_INV_FIXED != 0) ? (~cur18 + 18'sd1) : cur18);
                    blind_mm  <= 32'sd0;
                    turn_mode <= 2'd0;
                    hold_cnt  <= 16'd0;
                    DirOut    <= `DIR_FWD;
                    SpeedOut  <= 8'd0;
                    spd_r     <= 8'd0;
                    State     <= S_CRUISE;       
                end
                else if(hold_cnt >= 16'd300) begin      
                    State    <= S_ABORT;
                    ErrCode  <= 8'd1;                   
                end
                else hold_cnt <= hold_cnt + 16'd1;
            end
            else begin
            DirOut   <= `DIR_SPIN_L;
            SpeedOut <= CAL_SPEED;               
            spd_r    <= 8'd0;

            if(!yaw_fresh) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd2;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end

            else if(cal_seen || (hold_cnt >= CAL_CNT - 1)) begin
                yaw_inv    <= yaw_inv_nxt;
                fine_on    <= 1'b0;
                leg2nd     <= 1'b0;

                yaw_home   <= yaw_inv_nxt ? (~yaw_cal0 + 18'sd1) : yaw_cal0;

                turn_mode  <= 2'd0;
                turn_tgt   <= wrap18(yaw_inv_nxt ? (~yaw_cal0 + 18'sd1) : yaw_cal0);
                spin_lat   <= dir_for(wrap18(yaw_inv_nxt ? (~yaw_cal0 + 18'sd1) : yaw_cal0), yaw_ccw);
                hold_cnt   <= 16'd0;
                settle_cnt <= 16'd0;
                State      <= S_TURN;
            end
            else hold_cnt <= hold_cnt + 16'd1;
            end    
        end

        S_TURN: begin
            Active   <= 1'b1;
            DirOut   <= (spin_lat == `DIR_SPIN_L) ? `DIR_LEFT : `DIR_RIGHT;
            SpeedOut <= ARC_SPEED;
            spd_r    <= 8'd0;
            settle_cnt <= 16'd0;

            if(fwd_move) TotalMm <= TotalMm + dist_ext;    

            if(turn_done || ((turn_mode == 2'd2) && (hold_cnt >= arc_ticks))) begin
                hold_cnt <= 16'd0;
                case(turn_mode)

                2'd0: begin
                    blind_mm <= 32'sd0;
                    DirOut   <= `DIR_FWD;
                    spd_r    <= SLOW_SPEED;
                    SpeedOut <= SLOW_SPEED;
                    State    <= S_CRUISE;
                end

                2'd1: begin

                    turn_mode <= 2'd2;
                    spin_lat  <= (spin_lat == `DIR_SPIN_L) ? `DIR_SPIN_R : `DIR_SPIN_L;
                    turn_tgt  <= yaw_home;

                    arc_ticks <= arc_ticks_nxt;
                    hold_cnt  <= 16'd0;
                    State     <= S_TURN;
                end

                2'd2: begin

                    not_aligned <= 1'b0;
                    trim_cnt <= 2'd0;
                    blind_mm <= BLIND_S;
                    DirOut   <= `DIR_FWD;

                    spd_r    <= SLOW_SPEED;
                    SpeedOut <= SLOW_SPEED;
                    if(obs_cnt == 1'b0) begin
                        obs_cnt <= 1'b1;
                        State   <= S_CRUISE;       
                    end
                    else begin
                        Leg5Mm  <= 32'sd0;
                        State   <= S_RUN5;         
                    end
                end

                2'd3: begin
                    not_aligned <= 1'b0;
                    trim_cnt    <= 2'd0;
                    Leg6Mm      <= 32'sd0;
                    Leg7Mm      <= 32'sd0;
                    Leg8Mm      <= 32'sd0;
                    DirOut      <= `DIR_FWD;
                    spd_r       <= SLOW_SPEED;
                    SpeedOut    <= SLOW_SPEED;
                    settle_cnt  <= 16'd0;
                    hold_cnt    <= 16'd0;
                    trim_go     <= 1'b0;
                    pass_cnt    <= 4'd0;
                    if(TAIL_TRIM_EN != 0)
                        State <= S_TAILTRIM;     
                    else
                        case(leg_sel)
                        3'd4:    State <= S_RUN6;
                        3'd5:    State <= S_RUN7;
                        default: State <= S_RUN8;
                        endcase
                end
                endcase
            end
            else if(!yaw_fresh) begin

                hold_cnt <= 16'd0;
                State    <= S_ALIGN;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else if(hold_cnt >= TURN_CNT - 1) begin

                hold_cnt <= 16'd0;
                State    <= S_ALIGN;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else hold_cnt <= hold_cnt + 16'd1;
        end

        S_ALIGN: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_STOP;
            SpeedOut <= 8'd0;
            spd_r    <= 8'd0;

            if(settle_cnt >= SETTLE_CNT - 1) begin
                settle_cnt <= 16'd0;
                hold_cnt   <= 16'd0;
                spd_r      <= SLOW_SPEED;
                Leg2Mm     <= 32'sd0;
                Leg3Mm     <= 32'sd0;
                Leg4Mm     <= 32'sd0;
                Leg5Mm     <= 32'sd0;
                Leg6Mm     <= 32'sd0;
                Leg7Mm     <= 32'sd0;
                Leg8Mm     <= 32'sd0;
                DirOut     <= `DIR_FWD;
                SpeedOut   <= SLOW_SPEED;
                case(leg_sel)
                3'd0:    State <= S_RUN2;    
                3'd1:    State <= S_RUN3;    
                3'd2:    State <= S_RUN4;    
                3'd3:    State <= S_RUN5;    
                3'd4:    State <= S_RUN6;    
                3'd5:    State <= S_RUN7;    
                default: State <= S_RUN8;    
                endcase
            end
            else settle_cnt <= settle_cnt + 16'd1;
        end

        S_RUN2: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_FWD;

            if(spd_next >= CRUISE_W) begin
                spd_r    <= CRUISE_SPEED;
                SpeedOut <= CRUISE_SPEED;
            end
            else begin
                spd_r    <= spd_next[7:0];
                SpeedOut <= spd_next[7:0];
            end

            if(fwd_move) begin
                Leg2Mm  <= Leg2Mm  + dist_ext;
                TotalMm <= TotalMm + dist_ext;
            end

            if(f_guard && (Leg2Mm > 32'sd120)) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd6;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else if(Leg2Mm >= (PASS_S - LEAD_S)) begin

                leg_sel    <= 3'd1;

                turn_mode  <= 2'd2;         

                spin_lat   <= dir_for(yaw_home, yaw_ccw);
                turn_tgt   <= yaw_home;
                fine_on    <= 1'b0;
                Leg3Mm     <= 32'sd0;
                settle_cnt <= 16'd0;
                hold_cnt   <= 16'd0;
                spd_r      <= 8'd0;
                DirOut     <= `DIR_STOP;
                SpeedOut   <= 8'd0;
                State      <= S_TURN;
            end
            else if(seg_cnt >= SEG_CNT - 1) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd4;          
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
        end

        S_RUN3: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_FWD;
            if(spd_next >= CRUISE_W) begin
                spd_r    <= CRUISE_SPEED;
                SpeedOut <= CRUISE_SPEED;
            end
            else begin
                spd_r    <= spd_next[7:0];
                SpeedOut <= spd_next[7:0];
            end

            if(fwd_move) begin
                Leg3Mm  <= Leg3Mm  + dist_ext;
                TotalMm <= TotalMm + dist_ext;
            end

            if(f_guard && (Leg3Mm > 32'sd120)) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd6;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else if(Leg3Mm >= (D3_S - LEAD_S)) begin

                phase2   <= 1'b1;
                hold_cnt <= 16'd0;
                seg_cnt  <= 32'sd0;
                Leg4Mm   <= 32'sd0;
                Leg5Mm   <= 32'sd0;
                spd_r    <= 8'd0;
                DirOut   <= `DIR_FWD;
                SpeedOut <= CRUISE_SPEED;
                State    <= S_CRUISE;
            end
            else if(seg_cnt >= SEG_CNT - 1) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd4;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
        end

        S_RUN4: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_FWD;
            if(spd_next >= CRUISE_W) begin
                spd_r    <= CRUISE_SPEED;
                SpeedOut <= CRUISE_SPEED;
            end
            else begin
                spd_r    <= spd_next[7:0];
                SpeedOut <= spd_next[7:0];
            end

            if(fwd_move) begin
                Leg4Mm  <= Leg4Mm  + dist_ext;
                TotalMm <= TotalMm + dist_ext;
            end

            if(f_guard && (Leg4Mm > 32'sd120)) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd6;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else if(Leg4Mm >= (D4_S - LEAD_S)) begin

                leg_sel    <= 3'd3;
                turn_tgt   <= yaw_home;
                spin_lat   <= dir_for(yaw_home, yaw_ccw);
                leg2nd     <= 1'b1;
                settle_cnt <= 16'd0;
                hold_cnt   <= 16'd0;
                spd_r      <= 8'd0;
                DirOut     <= `DIR_STOP;
                SpeedOut   <= 8'd0;
                State      <= S_TURN;
            end
            else if(seg_cnt >= SEG_CNT - 1) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd4;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
        end

        S_RUN5: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_FWD;
            if(spd_next >= CRUISE_W) begin
                spd_r    <= CRUISE_SPEED;
                SpeedOut <= CRUISE_SPEED;
            end
            else begin
                spd_r    <= spd_next[7:0];
                SpeedOut <= spd_next[7:0];
            end

            if(fwd_move) begin
                Leg5Mm  <= Leg5Mm  + dist_ext;
                TotalMm <= TotalMm + dist_ext;
                if(blind_mm > 32'sd0) blind_mm <= blind_mm - dist_ext;
            end

            if((TAIL_ALWAYS_TURN == 0) && f_guard && (Leg5Mm > 32'sd120)) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd6;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else if(Leg5Mm >= (D5_S - LEAD_S)) begin

                leg_sel    <= 3'd4;
                turn_mode  <= 2'd3;                
                turn_tgt   <= wrap18(yaw_home + TURN_ANG_S);
                spin_lat   <= (TURN_ANG_S > 18'sd0) ? `DIR_SPIN_L : `DIR_SPIN_R;
                settle_cnt <= 16'd0;
                hold_cnt   <= 16'd0;
                seg_cnt    <= 16'd0;               

                DirOut     <= `DIR_FWD;
                SpeedOut   <= ARC_SPEED;
                State      <= S_TURN;
            end
            else if(seg_cnt >= SEG_CNT - 1) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd4;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
        end

        S_RUN6: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_FWD;
            if(spd_next >= CRUISE_W) begin
                spd_r    <= CRUISE_SPEED;
                SpeedOut <= CRUISE_SPEED;
            end
            else begin
                spd_r    <= spd_next[7:0];
                SpeedOut <= spd_next[7:0];
            end

            if(fwd_move) begin
                Leg6Mm  <= Leg6Mm  + dist_ext;
                TotalMm <= TotalMm + dist_ext;
            end

            if(f_guard && (Leg6Mm > 32'sd120)) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd6;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else if(Leg6Mm >= (D6_S - LEAD_S)) begin

                leg_sel    <= 3'd5;
                turn_mode  <= 2'd3;                
                turn_tgt   <= wrap18(yaw_home + TURN_2S);
                spin_lat   <= (TURN_ANG_S > 18'sd0) ? `DIR_SPIN_L : `DIR_SPIN_R;
                settle_cnt <= 16'd0;
                hold_cnt   <= 16'd0;

                DirOut     <= `DIR_FWD;
                SpeedOut   <= ARC_SPEED;
                State      <= S_TURN;
            end
            else if(seg_cnt >= SEG_CNT - 1) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd4;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
        end

        S_RUN7: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_FWD;
            if(spd_next >= CRUISE_W) begin
                spd_r    <= CRUISE_SPEED;
                SpeedOut <= CRUISE_SPEED;
            end
            else begin
                spd_r    <= spd_next[7:0];
                SpeedOut <= spd_next[7:0];
            end

            if(fwd_move) begin
                Leg7Mm  <= Leg7Mm  + dist_ext;
                TotalMm <= TotalMm + dist_ext;
            end

            if(f_guard && (Leg7Mm > 32'sd120)) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd6;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else if(Leg7Mm >= (D7_S - LEAD_S)) begin

                leg_sel    <= 3'd6;
                turn_mode  <= 2'd3;                
                turn_tgt   <= wrap18(yaw_home + TURN_3S);
                spin_lat   <= (TURN_ANG_S > 18'sd0) ? `DIR_SPIN_L : `DIR_SPIN_R;
                settle_cnt <= 16'd0;
                hold_cnt   <= 16'd0;

                DirOut     <= `DIR_FWD;
                SpeedOut   <= ARC_SPEED;
                State      <= S_TURN;
            end
            else if(seg_cnt >= SEG_CNT - 1) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd4;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
        end

        S_RUN8: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_FWD;
            if(spd_next >= CRUISE_W) begin
                spd_r    <= CRUISE_SPEED;
                SpeedOut <= CRUISE_SPEED;
            end
            else begin
                spd_r    <= spd_next[7:0];
                SpeedOut <= spd_next[7:0];
            end

            if(fwd_move) begin
                Leg8Mm  <= Leg8Mm  + dist_ext;
                TotalMm <= TotalMm + dist_ext;
            end

            if(f_guard && (Leg8Mm > 32'sd120)) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd6;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else if(Leg8Mm >= (D8_S - LEAD_S)) begin
                State    <= S_BRAKE;           
                hold_cnt <= 16'd0;
                spd_r    <= 8'd0;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else if(seg_cnt >= SEG_CNT - 1) begin
                State    <= S_ABORT;
                ErrCode  <= 8'd4;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
        end

        S_TAILTRIM: begin
            Active   <= 1'b1;

            if(!trim_go) begin

                DirOut   <= `DIR_FWD;
                SpeedOut <= SLOW_SPEED;
                spd_r    <= SLOW_SPEED;

                if(yaw_new) begin
                    trim_go <= 1'b1;
                    if((err_abs <= TRIM_DB_S) || (pass_cnt >= TAIL_TRIM_PASS)) begin

                        hold_cnt <= 16'd0;
                        case(leg_sel)
                        3'd4:    State <= S_RUN6;
                        3'd5:    State <= S_RUN7;
                        default: State <= S_RUN8;
                        endcase
                    end
                    else begin

                        trim_dir   <= yaw_err[17] ? `DIR_SPIN_R : `DIR_SPIN_L;
                        trim_ticks <= trim_n;
                        pass_cnt   <= pass_cnt + 4'd1;
                        hold_cnt   <= 16'd0;

                    end
                end
                else if(hold_cnt >= 16'd30) begin

                    hold_cnt <= 16'd0;
                    case(leg_sel)
                    3'd4:    State <= S_RUN6;
                    3'd5:    State <= S_RUN7;
                    default: State <= S_RUN8;
                    endcase
                end
                else hold_cnt <= hold_cnt + 16'd1;
            end
            else begin

                DirOut   <= (trim_dir == `DIR_SPIN_L) ? `DIR_LEFT : `DIR_RIGHT;
                SpeedOut <= ARC_SPEED;
                spd_r    <= 8'd0;

                if(fwd_move) TotalMm <= TotalMm + dist_ext;

                if(hold_cnt >= {12'd0, trim_ticks}) begin

                    hold_cnt <= 16'd0;
                    trim_go  <= 1'b0;
                    DirOut   <= `DIR_FWD;
                    spd_r    <= SLOW_SPEED;
                    SpeedOut <= SLOW_SPEED;
                end
                else hold_cnt <= hold_cnt + 16'd1;
            end
        end

        S_BRAKE: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_STOP;
            SpeedOut <= 8'd0;
            spd_r    <= 8'd0;

            if(hold_cnt >= BRAKE_CNT - 1) begin
                State    <= S_DONE;
                Done     <= 1'b1;
                hold_cnt <= 16'd0;
            end
            else hold_cnt <= hold_cnt + 16'd1;
        end

        S_DONE: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_STOP;
            SpeedOut <= 8'd0;

            if(start_lat) begin               
                Done       <= 1'b0;
                ErrCode    <= 8'd0;
                Leg2Mm     <= 32'sd0;
                TotalMm    <= 32'sd0;
                spd_r      <= 8'd0;
                settle_cnt <= 16'd0;

                leg_sel    <= 3'd0;
                phase2     <= 1'b0;

                obs_cnt    <= 1'b0;
                blind_mm   <= 32'sd0;
                turn_mode  <= 2'd0;
                not_aligned<= 1'b0;
                trim_cnt   <= 2'd0;
                yaw_home   <= wrap18(yaw_ccw);
                State      <= S_CRUISE;
            end
        end

        S_ABORT: begin
            Active   <= 1'b1;
            DirOut   <= `DIR_STOP;
            SpeedOut <= 8'd0;
            spd_r    <= 8'd0;

            if(start_lat) begin
                ErrCode    <= 8'd0;
                Leg2Mm     <= 32'sd0;
                TotalMm    <= 32'sd0;
                settle_cnt <= 16'd0;
                seg_cnt    <= 16'd0;

                leg_sel    <= 3'd0;
                phase2     <= 1'b0;
                State      <= S_CRUISE;
            end
            else if(abort_lat) begin
                State   <= S_IDLE;
                ErrCode <= 8'd0;
            end
        end

        default: State <= S_IDLE;
        endcase
    end

endmodule
