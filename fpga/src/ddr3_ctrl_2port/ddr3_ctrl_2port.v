/*
 * ddr3_ctrl_2port.v
 * 摄像头写入与显示读出双端口调度，连接FIFO和DDR3存储接口。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
module ddr3_ctrl_2port #(
    parameter integer LINE_WORDS = 1280,
    parameter integer FRAME_ROWS = 720,

    parameter integer ROW_FLIP_EN = 1,
    parameter integer DIAG_EN = 0
)(
    input           clk                 ,       
    input           pll_lock            ,
    input           clk_200m            ,       
    input           sys_rst_n           ,       
    output          init_calib_complete ,     
    output          pll_stop            , 

    input           rd_load             ,    
    input           wr_load             ,
    input           wr_frame_good       ,    
    input   [27:0]  app_addr_rd_min     ,    
    input   [27:0]  app_addr_rd_max     ,    
    input   [7:0]   rd_bust_len         ,    
    input   [27:0]  app_addr_wr_min     ,    
    input   [27:0]  app_addr_wr_max     ,    
    input   [7:0]   wr_bust_len         ,    

    input           wr_clk              , 
    input           wfifo_wren          ,  
    input   [15:0]  wfifo_din           ,  
    output          wrfifo_full         ,
    input           rd_clk              ,  
    input           rfifo_rden          ,  
    output          rdfifo_empty        ,
    output [7:0] diag_bad_gray, diag_commit_gray, diag_drop_gray,
    output video_ready,
    output  [15:0]  rfifo_dout          ,  

    inout   [15:0]     ddr3_dq             ,    
    inout   [1:0]      ddr3_dqs_n          ,    
    inout   [1:0]      ddr3_dqs_p          ,    
    output  [13:0]     ddr3_addr           ,    
    output  [2:0]      ddr3_ba             ,    
    output             ddr3_ras_n          ,    
    output             ddr3_cas_n          ,    
    output             ddr3_we_n           ,    
    output             ddr3_reset_n        ,    
    output  [0:0]      ddr3_ck_p           ,    
    output  [0:0]      ddr3_ck_n           ,    
    output  [0:0]      ddr3_cke            ,    
    output  [0:0]      ddr3_cs_n           ,    
    output  [1:0]      ddr3_dm             ,    
    output  [0:0]      ddr3_odt                 
);
    wire    ui_clk;
    wire    ui_clk_sync_rst;
    wire    app_rdy;
    wire    app_wdf_rdy;
    wire    app_rd_data_valid;
    wire [127:0] app_rd_data;
    wire [27:0] app_addr;
    wire app_en;
    wire app_wdf_wren;
    wire app_wdf_end;
    wire [2:0] app_cmd;
    wire [127:0] app_wdf_data;

    wire app_rd_data_end, app_sr_active, app_ref_ack;

    fifo_ddr3_adapter #(
       .LINE_WORDS(LINE_WORDS),
       .FRAME_ROWS(FRAME_ROWS),
       .ROW_FLIP_EN(ROW_FLIP_EN),
       .DIAG_EN(DIAG_EN)
    ) fifo_ddr3_adapter(
       .ui_clk(ui_clk)              ,     
       .rst_n(sys_rst_n)               ,     
       .init_calib_complete(init_calib_complete) ,    
       .app_rdy(app_rdy)             ,    
       .app_wdf_rdy(app_wdf_rdy)         ,    
       .app_rd_data_valid(app_rd_data_valid)   ,    
       .app_rd_data(app_rd_data)         ,
       .app_addr(app_addr)            ,    
       .app_en(app_en)              ,    
       .app_wdf_wren(app_wdf_wren)        ,    
       .app_wdf_end(app_wdf_end)         ,    
       .app_cmd(app_cmd)             ,    
       .app_wdf_data(app_wdf_data)        ,    
       .rd_load(rd_load)             ,    
       .wr_frame_good(wr_frame_good),
       .diag_bad_gray(diag_bad_gray),.diag_commit_gray(diag_commit_gray),
       .diag_drop_gray(diag_drop_gray),.video_ready(video_ready),
       .wr_load(wr_load)             ,    
       .app_addr_rd_min(app_addr_rd_min)     ,    
       .app_addr_rd_max(app_addr_rd_max)     ,    
       .rd_bust_len(rd_bust_len)         ,    
       .app_addr_wr_min(app_addr_wr_min)     ,    
       .app_addr_wr_max(app_addr_wr_max)     ,    
       .wr_bust_len(wr_bust_len)         ,    
       .wr_clk(wr_clk)              , 
       .wfifo_wren(wfifo_wren)          ,  
       .wfifo_din(wfifo_din)           ,  
       .wrfifo_full(wrfifo_full)       ,
       .rd_clk(rd_clk)              ,  
       .rfifo_rden(rfifo_rden)          ,  
       .rdfifo_empty(rdfifo_empty)      ,
       .rfifo_dout(rfifo_dout)            
    );

    DDR3_Memory_Interface_Top DDR3_Memory_Interface_Top(
        .clk(clk),  
		.pll_stop(pll_stop),  
        .memory_clk(clk_200m),  
        .pll_lock(pll_lock),  
        .rst_n(sys_rst_n),  
        .cmd_ready(app_rdy),  
        .cmd(app_cmd),  
        .cmd_en(app_en),  
        .addr(app_addr),  
        .wr_data_rdy(app_wdf_rdy),  
        .wr_data(app_wdf_data),  
        .wr_data_en(app_wdf_wren),  
        .wr_data_end(app_wdf_end),  
        .wr_data_mask(16'b0),  
        .rd_data(app_rd_data),  
        .rd_data_valid(app_rd_data_valid),  
        .rd_data_end(app_rd_data_end),  
        .sr_req(1'b0),  
        .ref_req(1'b0),  
        .sr_ack(app_sr_active),  
        .ref_ack(app_ref_ack),  
        .init_calib_complete(init_calib_complete),  
        .clk_out(ui_clk),  
        .ddr_rst(ui_clk_sync_rst),  
        .burst(1),  
        .O_ddr_addr(ddr3_addr),  
        .O_ddr_ba(ddr3_ba),  
        .O_ddr_cs_n(ddr3_cs_n),  
        .O_ddr_ras_n(ddr3_ras_n),  
        .O_ddr_cas_n(ddr3_cas_n),  
        .O_ddr_we_n(ddr3_we_n),  
        .O_ddr_clk(ddr3_ck_p),  
        .O_ddr_clk_n(ddr3_ck_n),  
        .O_ddr_cke(ddr3_cke),  
        .O_ddr_odt(ddr3_odt),  
        .O_ddr_reset_n(ddr3_reset_n),  
        .O_ddr_dqm(ddr3_dm),  
        .IO_ddr_dq(ddr3_dq),  
        .IO_ddr_dqs(ddr3_dqs_p),  
        .IO_ddr_dqs_n(ddr3_dqs_n)  
    );

endmodule 
