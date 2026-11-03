/*
 * tb_task4_seg1_tail.v
 * RTL测试平台：验证task4_seg1_tail的接口、边界条件或算法等价性。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`timescale 1ns/1ps
`include "car_defines.vh"

module tb_task4_seg1_tail;

    localparam integer TICK_DIV  = 20;    

    localparam integer ARC_SCALE = 1;
    localparam integer YAW_EVERY = 2;     

    localparam [7:0] S_IDLE=0, S_CRUISE=1, S_SLOW=2, S_STOP=3,
                     S_TURN=4, S_ALIGN=5, S_RUN2=6, S_BRAKE=7,
                     S_DONE=8, S_ABORT=9, S_CAL=10, S_RUN3=11,
                     S_RUN4=12, S_RUN5=13, S_RUN6=14, S_RUN7=15, S_RUN8=16,
                     S_TAILTRIM=17;    

    localparam integer D6_EXP = 600;    
    localparam integer D7_EXP = 1000;    
    localparam integer D8_EXP = 500;

    reg         clk = 0;
    reg         Rst_n = 0;
    reg         En = 0, Start = 0, Abort = 0, Estop = 0;
    reg  [7:0]  CmdDir = 0, CmdSpeed = 0;
    reg  [15:0] RangeF = 16'd2000;
    reg  signed [15:0] DistMm = 0, CurYaw = 0;
    reg         YawValid = 0;

    wire [7:0]  DirOut, SpeedOut, State, ErrCode;
    wire        Active, Done, seg1_beep;
    wire signed [31:0] Leg2Mm, TotalMm;
    wire signed [15:0] YawErrD100;
    wire        Tick;

    task4_seg1_fsm dut (
        .Clk (clk), .Rst_n (Rst_n),
        .En (En), .Start (Start), .Abort (Abort), .Estop (Estop), .Tick (Tick),
        .RangeF (RangeF), .DistMm (DistMm), .CurYaw (CurYaw), .YawValid (YawValid),
        .CmdDir (CmdDir), .CmdSpeed (CmdSpeed),
        .DirOut (DirOut), .SpeedOut (SpeedOut), .Active (Active), .State (State),
        .Done (Done), .Leg2Mm (Leg2Mm), .TotalMm (TotalMm),
        .YawErrD100 (YawErrD100), .ErrCode (ErrCode), .Beep (seg1_beep)
    );

    always #10 clk = ~clk;               

    reg [31:0] tdiv = 0;
    reg        tick_r = 0;
    always @(posedge clk) begin
        tdiv   <= (tdiv == TICK_DIV-1) ? 32'd0 : tdiv + 32'd1;
        tick_r <= (tdiv == TICK_DIV-1);
    end
    assign Tick = tick_r;

    reg [7:0] ycnt = 0;
    always @(posedge clk) begin
        YawValid <= 1'b0;
        if(!Rst_n) ycnt <= 8'd0;
        else if(Tick) begin
            if(ycnt >= YAW_EVERY-1) begin
                ycnt     <= 8'd0;
                YawValid <= 1'b1;
            end
            else ycnt <= ycnt + 8'd1;
        end
    end

    reg  signed [31:0] pos_q100 = 0;
    reg  signed [31:0] pos_mm_d = 0;
    reg  signed [31:0] yaw_u    = 0;       
    reg         model_rst = 0;             
    wire signed [31:0] pos_mm   = pos_q100 / 100;

    always @(posedge clk) begin
        if(!Rst_n) begin
            pos_q100 <= 0; pos_mm_d <= 0; yaw_u <= 0; DistMm <= 0;
        end
        else if(Tick && model_rst) begin
            pos_q100 <= 0; pos_mm_d <= 0; yaw_u <= 0; DistMm <= 0;
            model_rst <= 1'b0;
        end
        else if(Tick) begin
            if(DirOut == `DIR_FWD)  pos_q100 <= pos_q100 + SpeedOut*5;
            if(DirOut == `DIR_BACK) pos_q100 <= pos_q100 - SpeedOut*5;
            if(DirOut == `DIR_LEFT || DirOut == `DIR_RIGHT)
                pos_q100 <= pos_q100 + ((SpeedOut*5*3) >> 2);

            pos_mm_d <= pos_mm;
            DistMm   <= pos_mm - pos_mm_d;

            if(DirOut == `DIR_SPIN_L)       yaw_u <= yaw_u - SpeedOut*4093;
            else if(DirOut == `DIR_SPIN_R)  yaw_u <= yaw_u + SpeedOut*4093;
            else if(DirOut == `DIR_LEFT)    yaw_u <= yaw_u - (((SpeedOut*4093)>>2)*ARC_SCALE);
            else if(DirOut == `DIR_RIGHT)   yaw_u <= yaw_u + (((SpeedOut*4093)>>2)*ARC_SCALE);
        end
    end

    function signed [15:0] wrap180;
        input signed [31:0] a;
        integer r;
        begin
            r = a % 36000;
            if(r >  18000) r = r - 36000;
            if(r < -18000) r = r + 36000;
            wrap180 = r[15:0];
        end
    endfunction

    always @(posedge clk) begin
        if(!Rst_n) CurYaw <= 16'sd0;
        else if(YawValid) CurYaw <= wrap180(yaw_u / 1000);
    end

    integer rng;

    reg        wall3_on = 0;
    integer    rng2;
    always @(posedge clk) begin
        if(!Rst_n) RangeF <= 16'd2000;
        else begin
            if(pos_mm < 1200)      rng = 1200 - pos_mm;
            else if(pos_mm < 3050) rng = 3050 - pos_mm;
            else                   rng = 9999;

            if(wall3_on && (pos_mm >= 3100)) rng2 = 3850 - pos_mm;
            else                             rng2 = 9999;
            if(rng2 < rng) rng = rng2;
            if(rng > 4000) rng = 4000;
            if(rng < 20)   rng = 20;
            RangeF <= rng[15:0];
        end
    end

    integer checks = 0, errors = 0;
    integer a1_err = 0, a2_err = 0, a3_err = 0, a4_err = 0;
    integer a6_err = 0;    
    integer a7_ticks = 0;  

    reg [7:0]  st_d     = S_IDLE;
    reg        tail_on  = 0;       
    reg        t1_end   = 0;       
    integer    turn_idx = 0;       

    integer    tail_turn = 0;      
    integer    leg_mark = 0;
    integer    leg_len  = 0;
    integer    dyaw     = 0;
    integer    yaw0     = 0;
    integer    a4_abs   = 0;
    integer    exp_dyaw = 0, a4_dev = 0;

    reg        rerun    = 0;
    integer    rerun_ok = 0, rerun_err = 0;
    integer    rerun_checked = 0;

    always @(posedge clk) begin
        if(!Rst_n) begin
            st_d    <= S_IDLE;
            tail_on  = 0;
            turn_idx = 0;
        end
        else if(Tick) begin

            if(!t1_end && (((st_d == S_RUN5) && (State == S_TURN)) || (State == S_RUN6)))
                tail_on = 1;

            if(((State == S_RUN6) || (State == S_RUN7) || (State == S_RUN8)) &&
               ((st_d == S_TURN) || (st_d == S_TAILTRIM))) leg_mark = TotalMm;

            if((st_d == S_RUN5) && (State != S_TURN) && (State != S_RUN5) &&
               (State != S_ABORT)) begin
                a2_err = a2_err + 1;
                $display("[A2 FAIL] RUN5 -> %0d（应直跳 S_TURN）", State);
            end

            if((State == S_TURN) && tail_on && (st_d == S_TURN) &&
               (DirOut != `DIR_LEFT) && (DirOut != `DIR_STOP)) begin
                a1_err = a1_err + 1;
                $display("[A1 FAIL] 追加段转弯 dir=%0d（应为 DIR_LEFT=3）", DirOut);
            end

            if(tail_on && ((st_d == S_RUN5) || (st_d == S_RUN6) || (st_d == S_RUN7)) &&
               (State == S_TURN)) begin
                checks = checks + 1;
                if(SpeedOut != 8'd50) begin
                    a6_err = a6_err + 1;
                    $display("[A6 FAIL] 起弧前有停顿：SpeedOut=%0d（应为 50=ARC_SPEED）", SpeedOut);
                end
                else $display("  起弧无缝交接 SpeedOut=%0d DirOut=%0d  OK", SpeedOut, DirOut);
            end

            if(tail_on && !t1_end && (State == S_TAILTRIM) && (SpeedOut == 8'd0)) begin
                checks = checks + 1;
                a6_err = a6_err + 1;
                $display("[A8 FAIL] 航向微修正段出现 0 速（应保持 ARC_SPEED/SLOW_SPEED）");
            end

            if(!t1_end && tail_on && (st_d == S_RUN6) && (State == S_TURN)) begin
                leg_len = TotalMm - leg_mark;
                checks = checks + 1;
                if((leg_len < D6_EXP-80) || (leg_len > D6_EXP+80)) begin
                    a3_err = a3_err + 1;
                    $display("[A3 FAIL] D6 实走 %0d mm（目标 %0d）", leg_len, D6_EXP);
                end
                else $display("  D6 = %0d mm  OK", leg_len);
            end
            if(!t1_end && tail_on && (st_d == S_RUN7) && (State == S_TURN)) begin
                leg_len = TotalMm - leg_mark;
                checks = checks + 1;
                if((leg_len < D7_EXP-80) || (leg_len > D7_EXP+80)) begin
                    a3_err = a3_err + 1;
                    $display("[A3 FAIL] D7 实走 %0d mm（目标 %0d）", leg_len, D7_EXP);
                end
                else $display("  D7 = %0d mm  OK", leg_len);
            end
            if(!t1_end && tail_on && (st_d == S_RUN8) && (State == S_BRAKE)) begin
                leg_len = TotalMm - leg_mark;
                checks = checks + 1;
                if((leg_len < D8_EXP-80) || (leg_len > D8_EXP+80)) begin
                    a3_err = a3_err + 1;
                    $display("[A3 FAIL] D8 实走 %0d mm（目标 %0d）", leg_len, D8_EXP);
                end
                else $display("  D8 = %0d mm  OK", leg_len);
            end

            if(!t1_end && tail_on && ((st_d == S_TURN) || (st_d == S_TAILTRIM)) &&
               ((State == S_RUN6) || (State == S_RUN7) || (State == S_RUN8))) begin
                turn_idx  = turn_idx + 1;
                tail_turn = tail_turn + 1;
            end

            if(!t1_end && tail_on &&
               (((st_d == S_RUN6) && (State == S_TURN))  ||
                ((st_d == S_RUN7) && (State == S_TURN))  ||
                ((st_d == S_RUN8) && (State == S_BRAKE)))) begin
                checks   = checks + 1;
                dyaw     = wrap180(CurYaw - yaw0);
                a4_abs   = (dyaw < 0) ? -dyaw : dyaw;
                exp_dyaw = (tail_turn == 2) ? 18000 : 9000;
                a4_dev   = a4_abs - exp_dyaw;
                if(a4_dev < 0) a4_dev = -a4_dev;
                if(a4_dev > 150) begin
                    a4_err = a4_err + 1;
                    $display("[A4 FAIL] 直行段%0d 航向 Δyaw=%0d（期望 %0d，偏差 %0d = %0d.%02d° > 1.5°）",
                             tail_turn, dyaw, exp_dyaw, a4_dev, a4_dev/100, a4_dev%100);
                end
                else $display("  直行段%0d 航向 Δyaw=%0d（偏差 %0d = %0d.%02d°）  OK",
                              tail_turn, dyaw, a4_dev, a4_dev/100, a4_dev%100);
            end

            if(tail_on && !t1_end && (State == S_ALIGN)) a7_ticks = a7_ticks + 1;

            if(rerun && !rerun_checked && (State == S_RUN6) && (st_d != S_RUN6)) begin
                rerun_checked = 1;
                if(State == S_RUN6)                   rerun_ok = 1;
                else begin
                    rerun_err = rerun_err + 1;
                    $display("[RERUN FAIL] 重跑第一次摆正后进了 %0d（应为 14=S_RUN6，段去向未复位）",
                             State);
                end
            end

            st_d <= State;
        end
    end

    task do_reset;
        begin
            Rst_n = 0; En = 0; Start = 0; Abort = 0; Estop = 0;
            CmdDir = 0; CmdSpeed = 0;
            repeat(8) @(posedge clk);
            Rst_n = 1;
            repeat(8) @(posedge clk);
        end
    endtask

    task pulse_start;
        begin
            @(posedge clk);
            while(Tick) @(posedge clk);
            Start = 1'b1;
            @(posedge clk);
            Start = 1'b0;
        end
    endtask

    task wait_state;
        input  [7:0]   st;
        input  integer maxt;
        output integer ok;
        integer k;
        begin
            ok = 0; k = 0;
            while((k < maxt) && !ok) begin
                @(posedge clk);
                if(Tick) begin
                    k = k + 1;
                    if(State == st)   ok = 1;
                    if(State == S_ABORT) k = maxt;
                end
            end
        end
    endtask

    integer ok;

    initial begin
        $display("\n===== 尾段追加序列验证（终点不停车 + 三次左转）=====");
        do_reset;
        En = 1'b1;
        yaw0 = 0;
        pulse_start;

        wait_state(S_RUN6, 12000, ok);
        checks = checks + 1;
        if(!ok) begin
            errors = errors + 1;
            $display("[FAIL] 120s 内未进入 S_RUN6（state=%0d err=%0d total=%0d）",
                     State, ErrCode, TotalMm);
        end
        else $display("  已进入追加段 S_RUN6  OK  total=%0d", TotalMm);

        wait_state(S_DONE, 12000, ok);
        checks = checks + 1;
        if(!ok || (State != S_DONE) || (ErrCode != 0)) begin
            errors = errors + 1;
            $display("[A5 FAIL] state=%0d err=%0d ok=%0d（期望 DONE/0）", State, ErrCode, ok);
        end
        else $display("  A5 终态 DONE, ErrCode=0  OK");

        checks = checks + 1;
        if(tail_turn < 3) begin
            errors = errors + 1;
            $display("[FAIL] 追加段转弯次数=%0d（期望 3）", tail_turn);
        end
        else $display("  追加段转弯次数 = %0d  OK", tail_turn);

        checks = checks + 1;
        if(a7_ticks != 0) begin
            errors = errors + 1;
            $display("[A7 FAIL] 追加段共 %0d 拍停留在 S_ALIGN（转完应直接直行，不许停顿）",
                     a7_ticks);
        end
        else $display("  追加段未经过 S_ALIGN（转完无缝直行）  OK");

        checks = checks + 1;
        if((TotalMm < 6600) || (TotalMm > 8600)) begin
            errors = errors + 1;
            $display("[FAIL] TotalMm=%0d（D7 改 1000 后期望约 7100 ±700）", TotalMm);
        end
        else $display("  TotalMm = %0d mm  OK", TotalMm);

        errors = errors + a1_err + a2_err + a3_err + a4_err + a6_err;

        $display("\n===== T2 rerun after DONE =====");
        t1_end = 1;
        tail_on = 0;
        model_rst = 1'b1;                   
        repeat(4) @(posedge clk);
        rerun = 1;
        pulse_start;
        wait_state(S_RUN6, 12000, ok);
        checks = checks + 1;
        if(!ok || (State != S_RUN6) || (rerun_ok == 0) || (rerun_err != 0)) begin
            errors = errors + 1;
            $display("[RERUN FAIL] ok=%0d rerun_ok=%0d err=%0d state=%0d",
                     ok, rerun_ok, rerun_err, State);
        end
        else $display("  重跑后第一次摆正 → S_RUN6  OK（段去向已复位，未直接跳进 RUN8）");

        $display("\n===== T3 endpoint wall: must still turn left =====");
        wait_state(S_DONE, 20000, ok);           
        wall3_on  = 1;
        model_rst = 1'b1;
        rerun     = 0;
        repeat(4) @(posedge clk);
        pulse_start;
        wait_state(S_RUN6, 12000, ok);
        checks = checks + 1;
        if(!ok || (State != S_RUN6) || (ErrCode != 8'd0)) begin
            errors = errors + 1;
            $display("[T3 FAIL] state=%0d err=%0d ok=%0d（贴墙也必须左转）",
                     State, ErrCode, ok);
        end
        else $display("  终点贴墙仍进入 S_RUN6，ErrCode=0  OK");

        $display("\n---------------------------------------------");
        $display(" checks=%0d  errors=%0d  (A1=%0d A2=%0d A3=%0d A4=%0d A6=%0d rerun=%0d)",
                 checks, errors, a1_err, a2_err, a3_err, a4_err, a6_err, rerun_err);
        if(errors == 0) $display(" RESULT: TAIL SEQUENCE PASSED");
        else            $display(" RESULT: FAILED (%0d)", errors);
        $display("---------------------------------------------\n");
        $finish;
    end

endmodule
