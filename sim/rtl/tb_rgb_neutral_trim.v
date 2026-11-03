/*
 * tb_rgb_neutral_trim.v
 * RTL测试平台：验证rgb_neutral_trim的接口、边界条件或算法等价性。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`timescale 1ns/1ps
module tb_rgb_neutral_trim;
reg [15:0] din;
wire [23:0] dout;
integer i,r,g,b,er,eb;
rgb_neutral_trim dut(din,dout);
initial begin
 for(i=0;i<65536;i=i+1) begin
  din=i; r=((i>>11)&31); r=(r<<3)|(r>>2);
  g=((i>>5)&63); g=(g<<2)|(g>>4);
  b=i&31; b=(b<<3)|(b>>2);
  er=(r*45+16)/32; if(er>255) er=255;
  eb=(b*34+16)/32; if(eb>255) eb=255;
  #1;
  if(dout!=={er[7:0],g[7:0],eb[7:0]}) begin
   $display("FAIL %h %h",din,dout); $finish;
  end
 end
 $display("PASS all 65536 RGB565 words vs independent rounded/saturated gain reference");
 $finish;
end
endmodule
