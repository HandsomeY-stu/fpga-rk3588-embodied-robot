/*
 * DVP_Capture.v
 * DVP摄像头数据采集：按有效窗口组合RGB565字节并生成写缓存请求。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
module DVP_Capture(
  input            Rst_n,
  input            PCLK,
  input            Vsync,
  input            Href,
  input     [7:0]  Data,

  output           FrameGood,
  output reg       ImageState,
  output           DataValid,
  output    [15:0] DataPixel,
  output           DataHs,
  output           DataVs,
  output    [10:0] Xaddr,
  output    [10:0] Yaddr
);

  parameter BYTE_SWAP = 1;
  parameter integer FRAME_ROWS = 720;

  parameter USE_HCOUNT_LINE = 1;
  parameter [11:0] HACTIVE_BYTES = 12'd2560;
  parameter [5:0]  HREF_GAP_N    = 6'd48;

  parameter LINE_ALIGN_EN = 1;
  parameter [11:0] LINE_PX = 12'd1280;

  reg        r_Vsync_f;
  reg       r_Vsync;
  reg       r_Href;
  reg [7:0] r_Data;
  reg       r_Href_d;                  

  reg [15:0]r_DataPixel;
  reg       r_DataValid;
  reg       r_DataHs;
  reg       r_DataVs;
  reg [11:0]Hcount;
  reg [10:0]Vcount;
  reg [3:0] FrameCnt;

  reg       dump_frame;

  reg        in_line;
  wire       href_eff;
  reg [5:0]  href_low_cnt;

  reg [11:0] line_px_cnt;
  reg        line_flush;
  reg        href_eff_d;

  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    ImageState <= 1'b1;
  else if(r_Vsync_f)    
    ImageState <= 1'b0;

  always@(posedge PCLK)
  begin
    r_Vsync <= Vsync;
    r_Href  <= Href;
    r_Data  <= Data;
  end

  parameter [15:0] VSYNC_HIGH_N = 16'd64;

  reg [15:0] vsync_high_cnt;

  reg        r_Vsync_f_d;     

  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n) begin
    vsync_high_cnt <= 16'd0;
    r_Vsync_f      <= 1'b0;
    r_Vsync_f_d    <= 1'b0;
  end
  else begin
    r_Vsync_f_d <= r_Vsync_f;
    if(!r_Vsync) begin
      vsync_high_cnt <= 16'd0;
      r_Vsync_f      <= 1'b0;
    end
    else if(vsync_high_cnt >= VSYNC_HIGH_N) begin
      r_Vsync_f      <= 1'b1;
    end
    else
      vsync_high_cnt <= vsync_high_cnt + 1'b1;
  end

  wire vsync_start = r_Vsync_f & ~r_Vsync_f_d;    

  wire href_rise   = r_Href & ~r_Href_d;
  wire byte_full   = (Hcount >= (HACTIVE_BYTES - 12'd1));

  wire true_start  = USE_HCOUNT_LINE ? (~in_line & href_rise)
                                     : (r_Href & ~r_Href_d);

  wire line_start  = href_eff & ~href_eff_d;

  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n) begin
    in_line      <= 1'b0;
    href_low_cnt <= 6'd0;
    r_Href_d     <= 1'b0;
  end
  else begin
    r_Href_d <= r_Href;

    if(USE_HCOUNT_LINE) begin
      if(!in_line) begin
        href_low_cnt <= 6'd0;
        if(href_rise)
          in_line <= 1'b1;                        
      end
      else if(byte_full) begin
        in_line      <= 1'b0;                     
        href_low_cnt <= 6'd0;
      end
      else if(r_Href) begin
        href_low_cnt <= 6'd0;                     
      end
      else if(href_low_cnt >= HREF_GAP_N) begin
        in_line      <= 1'b0;                     
        href_low_cnt <= 6'd0;
      end
      else
        href_low_cnt <= href_low_cnt + 1'b1;      
    end
    else begin
      in_line <= r_Href;                          
    end
  end

  assign href_eff = USE_HCOUNT_LINE ? in_line : r_Href;

  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    Hcount <= 12'd0;
  else if(true_start)
    Hcount <= 12'd0;
  else if(href_eff)
    Hcount <= Hcount + 1'd1;

  wire px_emit = LINE_ALIGN_EN
               ? ((href_eff && Hcount[0] && (line_px_cnt < LINE_PX)) ||
                  (~href_eff && line_flush && (line_px_cnt < LINE_PX)))
               : (Hcount[0] && href_eff);

  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n) begin
    line_px_cnt <= 12'd0;
    line_flush  <= 1'b0;
    href_eff_d  <= 1'b0;
  end
  else begin
    href_eff_d <= href_eff;
    if(line_start)
      line_px_cnt <= 12'd0;
    else if(px_emit)
      line_px_cnt <= line_px_cnt + 1'b1;

    if(href_eff)
      line_flush <= 1'b0;
    else if(line_flush)
      line_flush <= (line_px_cnt < LINE_PX) && (line_px_cnt != 12'd0);
    else
      line_flush <= (line_px_cnt != 12'd0) && (line_px_cnt < LINE_PX);
  end

  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    r_DataPixel <= 0;
  else if(href_eff) begin
    if((!Hcount[0]) ^ BYTE_SWAP)
      r_DataPixel[15:8] <= r_Data;
    else
      r_DataPixel[7:0] <= r_Data;
  end

  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    r_DataValid <= 0;
  else
    r_DataValid <= px_emit;

  always@(posedge PCLK)
  begin
    r_DataHs <= r_Href;
    r_DataVs <= ~r_Vsync_f;    
  end

  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    Vcount <= 0;
  else if(r_Vsync_f)    
    Vcount <= 0;
  else if(line_start)
    Vcount <= Vcount + 1'd1;

  assign Yaddr = Vcount;
  assign Xaddr = Hcount[11:1];

  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    FrameCnt <= 0;
  else if(vsync_start)begin    
    if(FrameCnt >= 10)
      FrameCnt <= 4'd10;
    else
      FrameCnt <= FrameCnt + 1'd1;
  end

  always@(posedge PCLK or negedge Rst_n)
  if(!Rst_n)
    dump_frame <= 0;
  else if(FrameCnt >= 10)
    dump_frame <= 1'd1;
  else
    dump_frame <= 0;

  assign FrameGood = dump_frame;
  assign DataPixel = r_DataPixel;
  assign DataValid = r_DataValid & dump_frame;
  assign DataHs    = r_DataHs & dump_frame;
  assign DataVs    = r_DataVs & dump_frame;

endmodule

