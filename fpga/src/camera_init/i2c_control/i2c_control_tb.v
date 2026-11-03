/*
 * i2c_control_tb.v
 * RTL测试平台：验证i2c_control的事务和时序。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn

`timescale 1ns / 1ps

module i2c_control_tb;

  reg         Clk;
  reg         Rst_n;
  reg         wrreg_req;
  reg         rdreg_req;
  reg  [15:0] addr;
  reg         addr_mode;
  reg  [7:0]  wrdata;
  wire [7:0]  rddata;
  reg  [7:0]  device_id;
  wire        RW_Done;
  wire        ack;
  wire        i2c_sclk;
  wire        i2c_sdat;

  pullup PUP(i2c_sdat);

  i2c_control DUT(
    .Clk        (Clk      ),
    .Rst_n      (Rst_n    ),
    .wrreg_req  (wrreg_req),
    .rdreg_req  (rdreg_req),
    .addr       (addr     ),
    .addr_mode  (addr_mode),
    .wrdata     (wrdata   ),
    .rddata     (rddata   ),
    .device_id  (device_id),
    .RW_Done    (RW_Done  ),
    .ack        (ack      ),
    .dly_cnt_max(250-1    ),
    .i2c_sclk   (i2c_sclk ),
    .i2c_sdat   (i2c_sdat )
  );

  M24LC04B M24LC04B(
    .A0    (0        ),
    .A1    (0        ),
    .A2    (0        ),
    .WP    (0        ),
    .SDA   (i2c_sdat ),
    .SCL   (i2c_sclk ),
    .RESET (~Rst_n   )
  );

  M24LC64 M24LC64(
    .A0    (1        ),
    .A1    (0        ),
    .A2    (0        ),
    .WP    (0        ),
    .SDA   (i2c_sdat ),
    .SCL   (i2c_sclk ),
    .RESET (~Rst_n   )
  );

  initial Clk = 1;
  always #10 Clk = ~Clk;

  initial begin
    Rst_n = 0;
    rdreg_req = 0;
    wrreg_req = 0;
    #2001;
    Rst_n = 1;
    #2000;

    write_one_byte(0,8'hA0,16'h0A,8'hd1);
    write_one_byte(0,8'hA0,16'h0B,8'hd2);
    write_one_byte(0,8'hA0,16'h0C,8'hd3);
    write_one_byte(0,8'hA0,16'h0F,8'hd4);

    read_one_byte(0,8'hA0,16'h0A);
    read_one_byte(0,8'hA0,16'h0B);
    read_one_byte(0,8'hA0,16'h0C);
    read_one_byte(0,8'hA0,16'h0F);

    #2000000;

    write_one_byte(1,8'hA2,16'h030A,8'hd1);
    write_one_byte(1,8'hA2,16'h030B,8'hd2);
    write_one_byte(1,8'hA2,16'h030C,8'hd3);
    write_one_byte(1,8'hA2,16'h030F,8'hd4);

    read_one_byte(1,8'hA2,16'h030A);
    read_one_byte(1,8'hA2,16'h030B);
    read_one_byte(1,8'hA2,16'h030C);
    read_one_byte(1,8'hA2,16'h030F);
    $stop;  
  end

  task write_one_byte;
    input mode;
    input [7:0]id;
    input [15:0]mem_address; 
    input [7:0]data;
    begin
      addr = mem_address;
      device_id = id;
      addr_mode = mode;
      wrdata = data;
      wrreg_req = 1;
      #20;
      wrreg_req = 0;
      @(posedge RW_Done);
      #5000000;
    end
  endtask

  task read_one_byte;
    input mode;
    input [7:0]id;
    input [15:0]mem_address; 
    begin
      addr = mem_address;
      device_id = id;
      addr_mode = mode;
      rdreg_req = 1;
      #20;
      rdreg_req = 0;
      @(posedge RW_Done);
      #5000000;
    end
  endtask

endmodule
