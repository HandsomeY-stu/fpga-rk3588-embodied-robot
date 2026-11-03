/*
 * tb_task4_controls.v
 * RTL测试平台：验证task4_controls的接口、边界条件或算法等价性。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`timescale 1ns/1ps
`include "car_defines.vh"
module tb_task4_controls;
reg clk=0,rst=0,en=0,start=0,abort=0,estop=0,tick=0,yv=0;
reg [7:0] manual_dir=0,manual_spd=0;
wire [7:0] dir,spd,state,err;
wire active,done;
integer n=0;
always #10 clk=~clk;
always @(negedge clk) begin n=n+1; tick=(n%20==0); yv=(n%20==2); end
task4_seg1_fsm dut(.Clk(clk),.Rst_n(rst),.En(en),.Start(start),.Abort(abort),
 .Estop(estop),.Tick(tick),.RangeF(16'd2000),.DistMm(16'sd0),.CurYaw(16'sd0),
 .YawValid(yv),.CmdDir(manual_dir),.CmdSpeed(manual_spd),.DirOut(dir),.SpeedOut(spd),
 .Active(active),.State(state),.Done(done),.Leg2Mm(),.TotalMm(),.YawErrD100(),.ErrCode(err),.Beep());
task reset_dut;
begin rst=0;en=0;start=0;abort=0;estop=0;#85;rst=1;#100;end
endtask
task start_between_ticks;
begin
 en=1;
 wait(n%20==5); @(negedge clk);start=1;@(negedge clk);start=0;
 repeat(100) @(negedge clk);
 if(!active || dir!=`DIR_FWD || spd==0) $fatal(1,"lost asynchronous-to-Tick Start");
end
endtask
initial begin
 reset_dut; start_between_ticks;
 estop=1; @(posedge clk);#1;
 if(dir!=0 || spd!=0) $fatal(1,"Estop not immediate");
 reset_dut; start_between_ticks;
 wait(n%20==5);@(negedge clk);abort=1;@(negedge clk);abort=0;
 repeat(3) @(posedge clk);#1;
 if(dir!=0 || spd!=0 || state!=9) $fatal(1,"Abort pulse not latched");
 reset_dut; start_between_ticks;
 manual_dir=0;manual_spd=0;en=0;@(posedge clk);#1;
 if(active || dir!=0 || spd!=0) $fatal(1,"mode exit retained override");
 $display("PASS Start latch / Estop / Abort latch / mode-exit release");$finish;
end
initial begin #1000000;$fatal(1,"timeout");end
endmodule
