/*
 * tb_remote34.v
 * RTL测试平台：验证remote34的接口、边界条件或算法等价性。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`timescale 1ns/1ps
`include "car_defines.vh"

module tb_remote34;

    reg         Clk   = 1'b0;
    reg         Rst_n = 1'b0;
    always #10 Clk = ~Clk;                  

    reg  [7:0]  RxData  = 8'h00;
    reg         RxValid = 1'b0;
    reg         TxBusy  = 1'b0;

    wire [7:0]  TxData;
    wire        TxValid;
    wire        VisValid;
    wire [7:0]  VisCls, VisConf, VisSeq;
    wire [15:0] VisCx, VisCy, VisW, VisH;
    wire [15:0] PingCnt, PongCnt, VisCnt, ErrCnt;
    wire        LinkAlive;
    wire        MoValid, MoRejected, MoLinkLive;
    wire [7:0]  MoCode, MoSeq, MoLastCode;
    wire [15:0] MoArg0, MoArg1, MoArg2, MoGapMs;
    wire        StartT3, StartT4;

    vis_uart_bridge #(
        .CLK_FREQ        (50_000_000),
        .VIS_TIMEOUT_MS  (500),
        .LINK_TIMEOUT_MS (500),
        .ALLOW_ROTATE    (0),
        .REMOTE_DEDUP_MS (2)             
    ) dut (
        .Clk (Clk), .Rst_n (Rst_n),
        .RxData (RxData), .RxValid (RxValid),
        .TxData (TxData), .TxValid (TxValid), .TxBusy (TxBusy),
        .VisValid (VisValid), .VisCls (VisCls),
        .VisCx (VisCx), .VisCy (VisCy), .VisW (VisW), .VisH (VisH),
        .VisConf (VisConf), .VisSeq (VisSeq),
        .PingCnt (PingCnt), .PongCnt (PongCnt), .VisCnt (VisCnt),
        .LinkAlive (LinkAlive), .ErrCnt (ErrCnt),
        .MoValid (MoValid), .MoCode (MoCode),
        .MoArg0 (MoArg0), .MoArg1 (MoArg1), .MoArg2 (MoArg2),
        .MoSeq (MoSeq), .MoRejected (MoRejected), .MoLastCode (MoLastCode),
        .MoLinkLive (MoLinkLive), .MoGapMs (MoGapMs),
        .StartT3 (StartT3), .StartT4 (StartT4)
    );

    integer n_t3, n_t4, n_mo, n_rej, n_pong, n_err;
    integer fails;
    reg [7:0] last_mo_code;
    reg [7:0] last_mo_arg0;

    initial begin
        n_t3 = 0; n_t4 = 0; n_mo = 0; n_rej = 0; n_pong = 0; n_err = 0;
        fails = 0;
        last_mo_code = 8'hFF; last_mo_arg0 = 8'hFF;
    end

    always @(posedge Clk) begin
        if(StartT3) n_t3 <= n_t3 + 1;
        if(StartT4) n_t4 <= n_t4 + 1;
        if(MoValid) begin
            n_mo <= n_mo + 1;
            last_mo_code <= MoCode;
            last_mo_arg0 <= MoArg0[7:0];
        end
        if(MoRejected) n_rej <= n_rej + 1;
    end

    function [7:0] crc8b;
        input [7:0] c_in; input [7:0] b_in;
        integer i; reg [7:0] x;
        begin
            x = c_in ^ b_in;
            for (i = 0; i < 8; i = i + 1)
                x = x[7] ? ({x[6:0], 1'b0} ^ 8'h07) : {x[6:0], 1'b0};
            crc8b = x;
        end
    endfunction

    task send_byte;
        input [7:0] b;
        begin
            @(posedge Clk); RxData <= b; RxValid <= 1'b1;
            @(posedge Clk); RxValid <= 1'b0;
            repeat(30) @(posedge Clk);           
        end
    endtask

    task send_frame;
        input [7:0] cmd;
        input [7:0] d0;
        input [7:0] d1;
        input [7:0] nlen;
        input [7:0] bad_crc;                     
        reg [7:0] c;
        begin
            c = crc8b(`CRC8_INIT, nlen);
            c = crc8b(c, 8'h01);
            c = crc8b(c, cmd);
            if(nlen > 8'd1) c = crc8b(c, d0);
            if(nlen > 8'd2) c = crc8b(c, d1);
            if(bad_crc) c = c ^ 8'hFF;

            send_byte(`FRAME_HEAD_H);
            send_byte(`FRAME_HEAD_L);
            send_byte(nlen);
            send_byte(8'h01);
            send_byte(cmd);
            if(nlen > 8'd1) send_byte(d0);
            if(nlen > 8'd2) send_byte(d1);
            send_byte(c);
            repeat(300) @(posedge Clk);          
        end
    endtask

    task chk;
        input [255:0] name;
        input integer got;
        input integer exp;
        begin
            if(got === exp) $display("  [PASS] %0s : %0d", name, got);
            else begin
                $display("  [FAIL] %0s : got %0d, expect %0d", name, got, exp);
                fails = fails + 1;
            end
        end
    endtask

    initial begin
        $display("===== tb_remote34 : vis_uart_bridge 远程启动入口 =====");
        repeat(20) @(posedge Clk);
        Rst_n <= 1'b1;
        repeat(50) @(posedge Clk);

        $display("\n[T1] CMD_MODE(1=AVOID) 应只产生 1 拍 StartT3");
        send_frame(`CMD_MODE, `MODE_AVOID, 8'd0, 8'd2, 1'b0);
        chk("T1 StartT3", n_t3, 1);
        chk("T1 StartT4", n_t4, 0);
        chk("T1 MoValid ", n_mo, 0);             
        chk("T1 Rejected", n_rej, 0);            

        $display("\n[T2] 300ms 内重发 MODE(1) 应被去重");
        send_frame(`CMD_MODE, `MODE_AVOID, 8'd0, 8'd2, 1'b0);
        chk("T2 StartT3", n_t3, 1);

        $display("\n[T3] 等 >300ms 后发 MODE(4=SEG1) 应产生 StartT4");
        repeat(120_000) @(posedge Clk);           
        send_frame(`CMD_MODE, `MODE_SEG1,  8'd0, 8'd2, 1'b0);
        chk("T3 StartT4", n_t4, 1);
        chk("T3 StartT3", n_t3, 1);
        chk("T3 MoValid ", n_mo, 0);

        $display("\n[T4] CMD_MODE(0=MANUAL) 应进 MoValid 且 code=0x03/arg0=0");
        send_frame(`CMD_MODE, `MODE_MANUAL,8'd0, 8'd2, 1'b0);
        chk("T4 MoValid ", n_mo, 1);
        chk("T4 MoCode  ", last_mo_code, 8'h03);
        chk("T4 MoArg0  ", last_mo_arg0, 8'h00);
        chk("T4 StartT3 ", n_t3, 1);
        chk("T4 StartT4 ", n_t4, 1);

        $display("\n[T5] CMD_MODE(3=VISION) 应被拒绝且不触发启动");
        send_frame(`CMD_MODE, `MODE_VISION,8'd0, 8'd2, 1'b0);
        chk("T5 Rejected", n_rej, 1);
        chk("T5 MoValid ", n_mo, 1);
        chk("T5 StartT3 ", n_t3, 1);
        chk("T5 StartT4 ", n_t4, 1);

        $display("\n[T6] CMD_MOTION 应照常进 MoValid");
        send_frame(`CMD_MOTION, `DIR_FWD, 8'd60, 8'd3, 1'b0);    
        chk("T6 MoValid ", n_mo, 2);
        chk("T6 MoCode  ", last_mo_code, 8'h01);

        $display("\n[T7] CRC 错误帧不应产生任何动作");
        n_err = ErrCnt;
        send_frame(`CMD_MODE, `MODE_AVOID, 8'd0, 8'd2, 1'b1);    
        chk("T7 StartT3 ", n_t3, 1);
        chk("T7 MoValid ", n_mo, 2);
        if(ErrCnt > n_err) $display("  [PASS] T7 ErrCnt : %0d -> %0d", n_err, ErrCnt);
        else begin
            $display("  [FAIL] T7 ErrCnt 未增加 (%0d)", ErrCnt);
            fails = fails + 1;
        end

        $display("\n[T8] CMD_PING 应照常回 PONG（原有功能未受影响）");
        send_frame(`CMD_PING, 8'h00, 8'h00, 8'd1, 1'b0);

        chk("T8 PingCnt ", PingCnt, 1);

        $display("\n[T9] CMD_MODE(2=NAV) 也应产生 StartT4");
        repeat(120_000) @(posedge Clk);           
        send_frame(`CMD_MODE, `MODE_NAV,   8'd0, 8'd2, 1'b0);
        chk("T9 StartT4", n_t4, 2);
        chk("T9 StartT3", n_t3, 1);
        chk("T9 MoValid ", n_mo, 2);

        $display("\n===== 汇总：%0d 项失败 =====", fails);
        if(fails == 0) $display("RESULT: ALL_PASS");
        else           $display("RESULT: FAIL");
        $finish;
    end

    initial begin
        #50_000_000;                             
        $display("RESULT: TIMEOUT");
        $finish;
    end

endmodule
