/*
 * camera_init.v
 * 摄像头上电及寄存器配置状态机，向I2C接口顺序发送初始化表。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
// Company       : 武汉芯路恒科技有限公司
//                 http://xiaomeige.taobao.com
// Web           : http://www.corecourse.cn

module camera_init #(
  parameter SYS_CLOCK       = 50_000_000, 
  parameter SCL_CLOCK       = 400_000,    
  parameter CAMERA_TYPE     = "ov5640",   
  parameter IMAGE_TYPE      = 0,          
  parameter IMAGE_WIDTH     = 640,        
  parameter IMAGE_HEIGHT    = 480,        
  parameter IMAGE_FLIP_EN   = 0,
  parameter IMAGE_MIRROR_EN = 0
)
(
  input      Clk,
  input      Rst_n,

  output reg Init_Done,
  output     camera_rst_n,
  output     camera_pwdn,

  output     i2c_sclk,
  inout      i2c_sdat
);
  localparam RGB = 0;
  localparam JPEG = 1;
  localparam DELAY_1ms_CNT  = 1_000_000/(1_000_000_000/SYS_CLOCK);
  localparam DELAY_21ms_CNT = 21_000_000/(1_000_000_000/SYS_CLOCK);
  localparam DELAY_5ms_CNT  = 5_000_000/(1_000_000_000/SYS_CLOCK);

  wire [15:0]addr;
  reg wrreg_req;
  reg rdreg_req;
  wire [7:0]wrdata;
  wire [7:0]rddata;
  wire RW_Done;
  wire ack;
  reg [31:0] i2c_dly_cnt_max;

  reg [7:0]cnt;
  wire [23:0]lut;

  wire [7:0]lut_size;
  wire [7:0]device_id;
  wire addr_mode;

  assign camera_pwdn = 0;

  generate

  if(CAMERA_TYPE == "ov5640")
  begin
    assign device_id = 8'h78;
    assign addr_mode = 1'b1;
    assign addr = lut[23:8];
    assign wrdata = lut[7:0];

    if(IMAGE_TYPE == RGB)  
    begin
      assign lut_size = 253;

      ov5640_init_table_rgb #(
        .IMAGE_WIDTH     (IMAGE_WIDTH     ),
        .IMAGE_HEIGHT    (IMAGE_HEIGHT    ),
        .IMAGE_FLIP_EN   (IMAGE_FLIP_EN   ),
        .IMAGE_MIRROR_EN (IMAGE_MIRROR_EN )
      )ov5640_init_table_rgb_inst
      (
        .addr (cnt ),
        .clk  (Clk ),
        .q    (lut )
      );
    end
    else   
    begin
      assign lut_size = 250;

      ov5640_init_table_jpeg #(
        .IMAGE_WIDTH     (IMAGE_WIDTH     ),
        .IMAGE_HEIGHT    (IMAGE_HEIGHT    ),
        .IMAGE_FLIP_EN   (IMAGE_FLIP_EN   ),
        .IMAGE_MIRROR_EN (IMAGE_MIRROR_EN )
      )ov5640_init_table_jpeg_inst
      (
        .addr (cnt ),
        .clk  (Clk ),
        .q    (lut )
      );
    end
  end

  else if(CAMERA_TYPE == "ov7725")
  begin
    assign device_id = 8'h42;
    assign addr_mode = 1'b0;
    assign addr = lut[15:8];
    assign wrdata = lut[7:0];

    if(IMAGE_TYPE == RGB)  
    begin
      assign lut_size  = 68;

      ov7725_init_table_rgb #(
        .IMAGE_FLIP_EN   (IMAGE_FLIP_EN   ),
        .IMAGE_MIRROR_EN (IMAGE_MIRROR_EN )
      )ov7725_init_table_rgb_inst
      (
        .addr (cnt ),
        .clk  (Clk ),
        .q    (lut )
      );
    end
  end
  endgenerate

  i2c_control #(
    .SYS_CLOCK (SYS_CLOCK ),
    .SCL_CLOCK (SCL_CLOCK )
  )i2c_control(
    .Clk         (Clk             ),
    .Rst_n       (Rst_n           ),
    .wrreg_req   (wrreg_req       ),
    .rdreg_req   (1'd0            ),
    .addr        (addr            ),
    .addr_mode   (addr_mode       ),
    .wrdata      (wrdata          ),
    .rddata      (rddata          ),
    .device_id   (device_id       ),
    .RW_Done     (RW_Done         ),
    .ack         (ack             ),
    .dly_cnt_max (i2c_dly_cnt_max ),
    .i2c_sclk    (i2c_sclk        ),
    .i2c_sdat    (i2c_sdat        )
  );

  wire Go;    
  reg [20:0] delay_cnt;

  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)
    delay_cnt <= 21'd0;
  else if (delay_cnt >= DELAY_21ms_CNT)
    delay_cnt <= DELAY_21ms_CNT;
  else
    delay_cnt <= delay_cnt + 1'd1;

  assign Go = (delay_cnt == DELAY_21ms_CNT - 1'b1) ? 1'b1 : 1'b0;

  assign camera_rst_n = (delay_cnt > DELAY_1ms_CNT);

  localparam [7:0] RETRY_MAX = 8'd100;

  reg [7:0] retry_cnt;
  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)
    retry_cnt <= 8'd0;
  else if(Go)
    retry_cnt <= 8'd0;
  else if(RW_Done)
    retry_cnt <= (!ack) ? 8'd0 : (retry_cnt + 1'b1);

  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)
    cnt <= 0;
  else if(Go) 
    cnt <= 0;
  else if(cnt < lut_size)begin
    if(RW_Done && ((!ack) || (retry_cnt >= RETRY_MAX)))
      cnt <= cnt + 1'b1;       
    else
      cnt <= cnt;
  end
  else
    cnt <= 0;

  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)
    Init_Done <= 0;
  else if(Go) 
    Init_Done <= 0;
  else if(cnt == lut_size)
    Init_Done <= 1;

  reg [1:0]state;

  always@(posedge Clk or negedge Rst_n)
  if(~Rst_n)begin
    state <= 0;
    wrreg_req <= 1'b0;
    i2c_dly_cnt_max <= 32'd0;
  end
  else if(cnt < lut_size)begin
    case(state)
      0:
        if(Go)
          state <= 1;
        else
          state <= 0;
      1:
        begin
          wrreg_req <= 1'b1;
          state <= 2;
          if(cnt == 1)
            i2c_dly_cnt_max <= DELAY_5ms_CNT;  
          else
            i2c_dly_cnt_max <= 32'd0;
        end
      2:
        begin
          wrreg_req <= 1'b0;
          if(RW_Done)
            state <= 1;
          else
            state <= 2;
        end
      default:state <= 0;
    endcase
  end
  else
    state <= 0;

endmodule
