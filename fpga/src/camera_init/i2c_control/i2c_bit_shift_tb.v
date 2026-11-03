/*
 * i2c_bit_shift_tb.v
 * RTL测试平台：验证i2c_bit_shift的事务和时序。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn

`timescale 1ns / 1ps

module i2c_bit_shift_tb;

  reg         Clk;
  reg         Rst_n;
  reg  [5:0]  Cmd;
  reg         Go;
  wire [7:0]  Rx_DATA;
  reg  [7:0]  Tx_DATA;
  wire        Trans_Done;
  wire        ack_o;
  wire        i2c_sclk;
  wire        i2c_sdat;

  pullup PUP(i2c_sdat);

  localparam 
    WR   = 6'b000001,    
    STA  = 6'b000010,    
    RD   = 6'b000100,    
    STO  = 6'b001000,    
    ACK  = 6'b010000,    
    NACK = 6'b100000;    

  i2c_bit_shift DUT(
    .Clk        (Clk        ),
    .Rst_n      (Rst_n      ),
    .Cmd        (Cmd        ),
    .Go         (Go         ),
    .Rx_DATA    (Rx_DATA    ),
    .Tx_DATA    (Tx_DATA    ),
    .Trans_Done (Trans_Done ),
    .ack_o      (ack_o      ),
    .i2c_sclk   (i2c_sclk   ),
    .i2c_sdat   (i2c_sdat   )
  );

  M24LC04B M24LC04B(
    .A0   (0        ),
    .A1   (0        ),
    .A2   (0        ),
    .WP   (0        ),
    .SDA  (i2c_sdat ),
    .SCL  (i2c_sclk ),
    .RESET(~Rst_n   )
  );

  always #10 Clk = ~Clk;

  initial begin
    Clk = 1;
    Rst_n = 0;
    Cmd = 6'b000000;
    Go = 0;
    Tx_DATA = 8'd0;
    #2001;
    Rst_n = 1;
    #2000;

    Cmd = STA | WR;
    Go = 1;
    Tx_DATA = 8'hA0 | 8'd0; 
    @ (posedge Clk);
    Go = 0;
    @ (posedge Trans_Done);
    #200;

    Cmd = WR;
    Go = 1;
    Tx_DATA = 8'hB1; 
    @ (posedge Clk);
    Go = 0;
    @ (posedge Trans_Done);
    #200;

    Cmd = WR | STO;
    Go = 1;
    Tx_DATA = 8'hda; 
    @ (posedge Clk);
    Go = 0;
    @ (posedge Trans_Done);
    #200;

    #5000000;  

    Cmd = STA | WR;
    Go = 1;
    Tx_DATA = 8'hA0 | 8'd0; 
    @ (posedge Clk);
    Go = 0;
    @ (posedge Trans_Done);
    #200;

    Cmd = WR;
    Go = 1;
    Tx_DATA = 8'hB1; 
    @ (posedge Clk);
    Go = 0;
    @ (posedge Trans_Done);
    #200;

    Cmd = STA | WR;
    Go = 1;
    Tx_DATA = 8'hA0 | 8'd1; 
    @ (posedge Clk);
    Go = 0;
    @ (posedge Trans_Done);
    #200;

    Cmd = RD | STO;
    Go = 1;
    @ (posedge Clk);
    Go = 0;
    @ (posedge Trans_Done);
    #200;

    #2000;
    $stop;
  end

endmodule
