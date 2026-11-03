/*
 * tb_osd_mode_cdc.v
 * RTL测试平台：验证osd_mode_cdc的接口、边界条件或算法等价性。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`timescale 1ns/1ps
module tb_osd_mode_cdc;
reg sc=0,dc=0,rst=0,frame=0;
reg [1:0] desired=1;
wire [1:0] shown;
integer i,checks=0;
reg [1:0] previous=1;
reg prev_frame;
osd_mode_cdc dut(sc,dc,rst,desired,frame,shown);
always #10 sc=~sc;
always #6.667 dc=~dc;
always @(posedge dc) begin
    prev_frame=frame;
    #1;
    if(rst && shown!==previous && !prev_frame) $fatal(1,"mode changed during active frame");
    previous=shown;
end
task flush_check;
input [1:0] mode;
begin
    #800;
    @(negedge dc); frame=1;
    @(negedge dc); frame=0;
    #30;
    if(shown!==mode) $fatal(1,"wrong mode expected=%d got=%d",mode,shown);
    checks=checks+1;
end
endtask
initial begin
    #91; rst=1;
    flush_check(1);
    for(i=0;i<60;i=i+1) begin
        #17; desired=i%3; flush_check(i%3);
    end

    desired=2; #21; desired=0; #21; desired=1; #21; desired=2;
    flush_check(2);
    rst=0; desired=1; #87; rst=1; flush_check(1);
    $display("PASS OSD mailbox: %0d transitions/reset/coalescing; frame-boundary changes only",checks);
    $finish;
end
initial begin #1000000; $fatal(1,"timeout"); end
endmodule
