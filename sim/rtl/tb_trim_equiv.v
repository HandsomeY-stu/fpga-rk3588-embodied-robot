/*
 * tb_trim_equiv.v
 * RTL测试平台：验证trim_equiv的接口、边界条件或算法等价性。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`timescale 1ns/1ps
module tb_trim_equiv;
reg signed [17:0] stimulus;
reg signed [31:0] product;
reg [3:0] expected;
integer i;
task4_seg1_fsm dut();
initial begin
    force dut.err_abs=stimulus;
    for(i=0;i<262144;i=i+1) begin
        stimulus=i;
        product=($signed(stimulus)*32'sd10) >>> 10;
        expected=(product<1)?4'd1:((product>3)?4'd3:product[3:0]);
        #1;
        if(dut.trim_n!==expected) $fatal(1,"mismatch err=%d old=%d new=%d",stimulus,expected,dut.trim_n);
    end
    $display("PASS all 262144 signed 18-bit inputs: default v48 trim result unchanged");
    $finish;
end
endmodule
