/*
 * ov7725_init_table_rgb.v
 * OV7725备用寄存器表；本交付默认摄像头为OV5640。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn

module ov7725_init_table_rgb #(
  parameter DATA_WIDTH      = 16,
  parameter ADDR_WIDTH      = 8,
  parameter IMAGE_FLIP_EN   = 1'b0,
  parameter IMAGE_MIRROR_EN = 1'b0
)
(
  addr,
  clk, 
  q
);
  input clk;
  input [(ADDR_WIDTH-1):0] addr;
  output reg [(DATA_WIDTH-1):0] q;

  localparam IMAGE_FLIP_DAT   = IMAGE_FLIP_EN ? 1'b1 : 1'b0;
  localparam IMAGE_MIRROR_DAT = IMAGE_MIRROR_EN ? 1'b1 : 1'b0;

  reg [DATA_WIDTH-1:0] rom [2**ADDR_WIDTH-1:0];

  always@(*) begin

    rom[0 ] = 16'h12_80;  
    rom[1 ] = 16'h3d_03;  
    rom[2 ] = 16'h15_02;  
    rom[3 ] = 16'h17_22;  
    rom[4 ] = 16'h18_a4;  
    rom[5 ] = 16'h19_07;  
    rom[6 ] = 16'h1a_f0;  
    rom[7 ] = 16'h32_00;  
    rom[8 ] = 16'h29_A0;  
    rom[9 ] = 16'h2C_F0;  
    rom[10] = 16'h0d_41;  
    rom[11] = 16'h11_01;  

    rom[12] = 16'h12_06;  

    rom[13] = {8'h0c, IMAGE_MIRROR_DAT, IMAGE_FLIP_DAT, 2'd1, 4'd0};

    rom[14] = 16'h42_7f;  
    rom[15] = 16'h4d_09;  
    rom[16] = 16'h63_f0;  
    rom[17] = 16'h64_ff;  
    rom[18] = 16'h65_00;  
    rom[19] = 16'h66_00;  
    rom[20] = 16'h67_00;  

    rom[21] = 16'h13_ff;
    rom[22] = 16'h0f_c5;
    rom[23] = 16'h14_11;
    rom[24] = 16'h22_98;  
    rom[25] = 16'h23_03;  
    rom[26] = 16'h24_40;  
    rom[27] = 16'h25_30;  
    rom[28] = 16'h26_a1;  
    rom[29] = 16'h2b_9e;  
    rom[30] = 16'h6b_aa;  
    rom[31] = 16'h13_ff;  

    rom[32] = 16'h90_0a;
    rom[33] = 16'h91_01;
    rom[34] = 16'h92_01;
    rom[35] = 16'h93_01;
    rom[36] = 16'h94_5f;
    rom[37] = 16'h95_53;
    rom[38] = 16'h96_11;
    rom[39] = 16'h97_1a;
    rom[40] = 16'h98_3d;
    rom[41] = 16'h99_5a;
    rom[42] = 16'h9a_1e;
    rom[43] = 16'h9b_3f;  
    rom[44] = 16'h9c_25;
    rom[45] = 16'h9e_81;
    rom[46] = 16'ha6_06;
    rom[47] = 16'ha7_65;
    rom[48] = 16'ha8_65;
    rom[49] = 16'ha9_80;
    rom[50] = 16'haa_80;

    rom[51] = 16'h7e_0c;
    rom[52] = 16'h7f_16;  
    rom[53] = 16'h80_2a;
    rom[54] = 16'h81_4e;
    rom[55] = 16'h82_61;
    rom[56] = 16'h83_6f;
    rom[57] = 16'h84_7b;
    rom[58] = 16'h85_86;
    rom[59] = 16'h86_8e;
    rom[60] = 16'h87_97;
    rom[61] = 16'h88_a4;
    rom[62] = 16'h89_af;
    rom[63] = 16'h8a_c5;
    rom[64] = 16'h8b_d7;
    rom[65] = 16'h8c_e8;
    rom[66] = 16'h8d_20;

    rom[67] = 16'h0e_65;  
  end

  always @ (posedge clk)
  begin
    q <= rom[addr];
  end

endmodule
