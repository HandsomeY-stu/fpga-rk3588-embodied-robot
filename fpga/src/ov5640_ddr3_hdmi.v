/*
 * ov5640_ddr3_hdmi.v
 * FPGA系统顶层：连接摄像头、DDR3帧缓存、HDMI、IMU、运动控制及远程命令。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
module gao_ov5640_ddr_hdmi(

    input       clk50M        ,  
    input       reset_n       ,  

    output      camera_sclk   ,
    inout       camera_sdat   ,
    input       camera_vsync  ,
    input       camera_href   ,
    input       camera_pclk   ,
    output      camera_xclk   ,
    input  [7:0]camera_data   ,
    output      camera_rst_n  ,
    output      camera_pwdn   ,

    output [3:0]led           ,

    input        imu_rx       ,

    output       uart1_tx     ,
    input        uart1_rx     ,

    output [3:0] motor_pwm      ,  
    output [3:0] motor_dir      ,  
    output [3:0] motor_dirn     ,  

    input        enc_a0         ,  
    input        enc_b0         ,  
    input        enc_a1         ,  
    input        enc_b1         ,  

    output       us_trig0       ,  
    input        us_echo0       ,  
    output       us_trig1       ,  
    input        us_echo1       ,  
    input  [3:0] key            ,  
    output [3:0] bled           ,  
    output       beep           ,  

    output       seg_ds         ,  
    output       seg_shcp       ,  
    output       seg_stcp       ,  

    output       uart_tx        ,  

	output       	tmds_clk_p ,
	output       	tmds_clk_n ,
	output 	[2:0]   tmds_data_p,     
	output 	[2:0]   tmds_data_n,      

    output [13:0] O_ddr_addr        ,
    output [2:0] O_ddr_ba           ,
    output O_ddr_cs_n               ,
    output O_ddr_ras_n              ,
    output O_ddr_cas_n              ,
    output O_ddr_we_n               ,
    output O_ddr_clk                ,
    output O_ddr_clk_n              ,
    output O_ddr_cke                ,
    output O_ddr_odt                ,
    output O_ddr_reset_n            ,
    output [1:0] O_ddr_dqm          ,
    inout [15:0] IO_ddr_dq          ,
    inout [1:0] IO_ddr_dqs          ,
    inout [1:0] IO_ddr_dqs_n        

);

    parameter TASK1_DIAG_EN = 0;
    parameter IMAGE_WIDTH  = 1280;
    parameter IMAGE_HEIGHT = 720;
    parameter TEST_PATTERN_EN = 0;    

    wire          loc_clk24m;
    wire          g_rst_p;
    wire          loc_clk400m;
    wire          pll_lock;
    wire          loc_clk50m;
    wire          pll_locked;

    wire          camera_init_done;
    wire          pclk_bufg_o;

	wire fifo_aclr;
	wire fifo_wrreq;
	wire [15:0] fifo_wrdata;

	wire [15:0] fifo_wrdata_cam;      
	wire [15:0] cam_pixel_raw;        
	wire        cam_valid_raw;        
	wire [10:0] tp_x, tp_y;           
	reg  [15:0] tp_rgb;               
    wire init_calib_complete;
    wire wr_load;                     

    wire clk_vga; 
	wire clk_vgax5; 
    wire hdmi_pll_lock;

	wire [11:0] hcount;
	wire [11:0] vcount;
	wire [23:0] disp_data;

	wire video_hs;
	wire video_vs;
	wire video_de;
	wire [7:0] video_r;
	wire [7:0] video_g;
	wire [7:0] video_b;

    assign  g_rst_p = ~pll_locked;

        camera_pll your_instance_name(
        .clkin(clk50M),  
        .clkout0(clk_vgax5),  
        .clkout1(loc_clk24m),  
        .clkout2(loc_clk50m),  
        .lock(pll_locked),  
        .mdclk(clk50M),  
        .reset(~reset_n)  
    );

    CLKDIV u_clkdiv
    (.RESETN(pll_locked)
    ,.HCLKIN(clk_vgax5)  
    ,.CLKOUT(clk_vga)     
    ,.CALIB (1'b1)
    );
    defparam u_clkdiv.DIV_MODE="5";

    assign camera_xclk = loc_clk24m;

    localparam CAM_FLIP_EN   = 0;
    localparam CAM_MIRROR_EN = 0;

    camera_init
    #(
        .SYS_CLOCK      ( 50_000_000   ), 
        .SCL_CLOCK      ( 400_000      ), 
        .CAMERA_TYPE    ( "ov5640"     ), 
        .IMAGE_TYPE     ( 0            ), 
        .IMAGE_WIDTH    ( IMAGE_WIDTH  ), 
        .IMAGE_HEIGHT   ( IMAGE_HEIGHT ), 
        .IMAGE_FLIP_EN  ( CAM_FLIP_EN  ), 
        .IMAGE_MIRROR_EN( CAM_MIRROR_EN)  
    )camera_init
    (
        .Clk         (loc_clk50m       ),
        .Rst_n       (reset_n         ),
        .Init_Done   (camera_init_done ),
        .camera_rst_n(camera_rst_n     ),
        .camera_pwdn (camera_pwdn      ),
        .i2c_sclk    (camera_sclk      ),
        .i2c_sdat    (camera_sdat      )
    );

    assign pclk_bufg_o = camera_pclk;

    wire cam_frame_good;
    reg [7:0] capture_frame_id;
    reg capture_vs_d;
    always @(posedge pclk_bufg_o or negedge reset_n)
    if(!reset_n) begin capture_frame_id<=0;capture_vs_d<=0;end
    else begin
        capture_vs_d<=wr_load;
        if(wr_load && !capture_vs_d) capture_frame_id<=capture_frame_id+1'b1;
    end

    wire marker_on=TASK1_DIAG_EN && tp_y<32 && !tp_x[10];
    wire marker_bit=capture_frame_id[tp_x[9:7]] ^ tp_y[4];
    wire [15:0] camera_marked=marker_on ? (marker_bit ? 16'hffff : 16'h0000) : cam_pixel_raw;
    DVP_Capture #(.FRAME_ROWS(IMAGE_HEIGHT)) DVP_Capture(
        .Rst_n      (reset_n         ), 
        .PCLK       (pclk_bufg_o      ), 
        .Vsync      (camera_vsync     ), 
        .Href       (camera_href      ), 
        .Data       (camera_data      ), 

        .FrameGood  (cam_frame_good),
        .ImageState (fifo_aclr        ), 
        .DataValid  (cam_valid_raw    ), 
        .DataPixel  (cam_pixel_raw    ), 
        .DataHs     (                 ), 
        .DataVs     (wr_load          ), 
        .Xaddr      (tp_x             ), 
        .Yaddr      (tp_y             )    
    );

    parameter DESPECKLE_EN = 1;    

    wire [15:0] dsp_data;
    wire        dsp_valid;

    despeckle_median5 #(.LINE_PX(IMAGE_WIDTH)) u_despeckle (
        .PCLK   (pclk_bufg_o    ),    
        .Rst_n  (reset_n        ),
        .de     (cam_valid_raw  ),
        .din    (camera_marked  ),
        .dout   (dsp_data       ),
        .dvalid (dsp_valid      )
    );

    assign fifo_wrreq      = DESPECKLE_EN ? dsp_valid : cam_valid_raw;
    assign fifo_wrdata_cam = DESPECKLE_EN ? dsp_data  : camera_marked;

    always @(*) begin
        if (tp_y < 11'd360) begin                       
            if (tp_x[10])
                tp_rgb = 16'h0000;                      
            else case (tp_x[9:7])                       
                3'd0:    tp_rgb = 16'hFFFF;             
                3'd1:    tp_rgb = 16'hFFE0;             
                3'd2:    tp_rgb = 16'h07FF;             
                3'd3:    tp_rgb = 16'h07E0;             
                3'd4:    tp_rgb = 16'hF81F;             
                3'd5:    tp_rgb = 16'hF800;             
                3'd6:    tp_rgb = 16'h001F;             
                default: tp_rgb = 16'h0000;             
            endcase
        end
        else begin                                      

            if (tp_x[10] | tp_x[9])
                tp_rgb = (tp_x[0] ^ tp_y[0]) ? 16'hFFFF : 16'h0000;   
            else
                tp_rgb = 16'h8410;                      
        end
    end

    assign fifo_wrdata = TEST_PATTERN_EN ? tp_rgb : fifo_wrdata_cam;

    wire rdfifo_rden;
    wire wrfifo_clr;
    wire [15:0] wrfifo_din;
    wire wrfifo_clk;
    wire wrfifo_wren;
    wire rdfifo_clr;
    wire rdfifo_clk;
    wire [15:0] rdfifo_dout;

    assign wrfifo_clr = fifo_aclr;
    assign wrfifo_clk = pclk_bufg_o;
    assign rdfifo_clr = 0;
    assign rdfifo_clk = clk_vga;
    assign wrfifo_wren = fifo_wrreq;
    assign wrfifo_din = fifo_wrdata;

    wire [23:0] trimmed_rgb888;
    rgb_neutral_trim u_rgb_neutral_trim(.din(rdfifo_dout), .dout(trimmed_rgb888));
    assign disp_data = !t1_ready_sync[1] ? 24'd0 : trimmed_rgb888;

    wire [27:0] app_addr_max = IMAGE_WIDTH * IMAGE_HEIGHT;
    wire [7:0] burst_len = IMAGE_WIDTH[10:3];

    wire [7:0] t1_bad_gray,t1_commit_gray,t1_drop_gray;
    wire t1_video_ready;
    reg [1:0] t1_ready_sync;
    always @(posedge clk_vga or negedge reset_n)
    if(!reset_n) t1_ready_sync<=0;
    else t1_ready_sync<={t1_ready_sync[0],t1_video_ready};
    wire mdrp_inc;     
    wire [1:0] mdrp_op;     
    wire [7:0] mdrp_wdata;     
    wire [7:0] mdrp_rdata;     
    wire pll_stop;     
    reg pll_stop_r;
    reg wr;

    reg  pll_lock_r;
    reg  pll_lock_rr;

    always@(posedge clk50M)begin
        pll_lock_r <= pll_lock;
        pll_lock_rr <= pll_lock_r;
    end

    always@(posedge clk50M)
        pll_stop_r <= pll_stop;

    pll_mDRP_intf u_pll_mDRP_intf(
        .clk(clk50M),
        .rst_n(1'b1),
        .pll_lock(pll_lock_rr),
        .wr(wr),
        .mdrp_inc(mdrp_inc),
        .mdrp_op(mdrp_op),
        .mdrp_wdata(mdrp_wdata),
        .mdrp_rdata(mdrp_rdata)
    );    

    always@(posedge clk50M)
    if(!pll_lock_rr)
        wr <= 1'b0;
    else if(~pll_stop & pll_stop_r)begin
        wr <= 1'b1;
    end
    else if(pll_stop & (~pll_stop_r))begin
        wr <= 1'b1;
    end
    else begin
        wr <= 1'b0;
    end

    ddr_pll ddr_pll(
        .clkin(clk50M),  
        .clkout0(),  
        .clkout1(),  
        .clkout2(loc_clk400m),  
        .lock(pll_lock),  
        .mdopc(mdrp_op),  
        .mdainc(mdrp_inc),  
        .mdwdi(mdrp_wdata),  
        .mdrdo(mdrp_rdata),  
        .pll_init_bypass(1'b0),  
        .mdclk(clk50M),  
        .reset(~reset_n)  
    );

    localparam CAM_ROWFLIP_EN = 1;

    ddr3_ctrl_2port #(
        .LINE_WORDS(IMAGE_WIDTH),
        .FRAME_ROWS(IMAGE_HEIGHT),
        .ROW_FLIP_EN(CAM_ROWFLIP_EN),
        .DIAG_EN(TASK1_DIAG_EN)
    ) ddr3_ctrl_2port(
        .clk(loc_clk50m)                 ,       
        .pll_lock(pll_lock)            ,
        .pll_stop(pll_stop)                ,
        .clk_200m(loc_clk400m)            ,       
        .sys_rst_n(reset_n)           ,       
        .init_calib_complete(init_calib_complete) ,     

        .rd_load(video_vs)             ,    
        .wr_frame_good(cam_frame_good),
        .diag_bad_gray(t1_bad_gray),.diag_commit_gray(t1_commit_gray),
        .diag_drop_gray(t1_drop_gray),.video_ready(t1_video_ready),
        .wr_load(wr_load)             ,    
        .app_addr_rd_min(28'd0)     ,    
        .app_addr_rd_max(app_addr_max)     ,    
        .rd_bust_len(burst_len)         ,    
        .app_addr_wr_min(28'd0)     ,    
        .app_addr_wr_max(app_addr_max)     ,    
        .wr_bust_len(burst_len)         ,    

        .wr_clk(wrfifo_clk)             , 
        .wfifo_wren(wrfifo_wren)          ,  
        .wfifo_din(wrfifo_din)           ,  
        .rd_clk(rdfifo_clk)              ,  
        .rfifo_rden(rdfifo_rden)          ,  
        .rfifo_dout(rdfifo_dout)          ,  

        .ddr3_dq(IO_ddr_dq)             ,    
        .ddr3_dqs_n(IO_ddr_dqs_n)          ,    
        .ddr3_dqs_p(IO_ddr_dqs)          ,    
        .ddr3_addr(O_ddr_addr)           ,    
        .ddr3_ba(O_ddr_ba)             ,    
        .ddr3_ras_n(O_ddr_ras_n)          ,    
        .ddr3_cas_n(O_ddr_cas_n)          ,    
        .ddr3_we_n(O_ddr_we_n)           ,    
        .ddr3_reset_n(O_ddr_reset_n)        ,    
        .ddr3_ck_p(O_ddr_clk)          ,    
        .ddr3_ck_n(O_ddr_clk_n)           ,    
        .ddr3_cke(O_ddr_cke)            ,    
        .ddr3_cs_n(O_ddr_cs_n)           ,    
        .ddr3_dm(O_ddr_dqm)             ,    
        .ddr3_odt(O_ddr_odt)                 
    );

    parameter DISP_FRAME_BAR_EN = 0;

    wire        disp_frame_begin;
    reg  [5:0]  disp_frame_cnt;

    always @(posedge clk_vga or negedge reset_n)
    if(!reset_n)
        disp_frame_cnt <= 6'd0;
    else if(disp_frame_begin)
        disp_frame_cnt <= disp_frame_cnt + 1'b1;

    wire [11:0] disp_bar_w  = {4'b0, disp_frame_cnt, 2'b00};   
    wire        disp_bar_on = DISP_FRAME_BAR_EN &&
                              (vcount >= 12'd20) && (vcount < 12'd80) &&
                              (hcount < disp_bar_w);

    reg [1:0]  cam_vs_sync;
    reg [5:0]  cam_frame_cnt;

    always @(posedge clk_vga or negedge reset_n)
    if(!reset_n) begin
        cam_vs_sync   <= 2'b00;
        cam_frame_cnt <= 6'd0;
    end
    else begin
        cam_vs_sync <= {cam_vs_sync[0], camera_vsync};
        if(cam_vs_sync == 2'b01)                  
            cam_frame_cnt <= cam_frame_cnt + 1'b1;
    end

    wire [11:0] cam_bar_w  = {4'b0, cam_frame_cnt, 2'b00};
    wire        cam_bar_on = DISP_FRAME_BAR_EN &&
                             (vcount >= 12'd100) && (vcount < 12'd160) &&
                             (hcount < cam_bar_w);

    wire [23:0] disp_data_ov = disp_bar_on ? 24'hFF0000 :       
                               cam_bar_on  ? 24'h00C000 :       
                                             disp_data;

    parameter VER_MARK_EN = 0;    
    wire        ver_mark_on  = VER_MARK_EN && (hcount < 12'd24) && (vcount < 12'd24);
    wire [23:0] disp_data_mk = ver_mark_on ? 24'hFF0000 : disp_data_ov;

    parameter IMU_OSD_EN     = 1;
    parameter IMU_OSD_DBG_EN = 0;    

    localparam integer OSD_TXT_COLS  = 13;
    localparam integer OSD_TXT_ROWS  = 4;
    localparam integer OSD_DISP_ROWS = IMU_OSD_DBG_EN ? 4 : 3;
    localparam integer OSD_TXT_W     = OSD_TXT_ROWS * OSD_TXT_COLS * 8;

    wire [7:0]         imu_byte;
    wire               imu_byte_v;
    wire signed [15:0] imu_roll, imu_pitch, imu_yaw;       
    wire signed [15:0] att_roll, att_pitch, att_yaw;       
    wire               imu_ang_v, imu_link;
    wire [15:0]        imu_ok_cnt, imu_err_cnt;
    wire [63:0]        imu_dbg_bytes;    
    wire [OSD_TXT_W-1:0] imu_txt_a, imu_txt_b;
    wire               imu_bank;

    uart_rx_8n1 #(
        .CLK_FREQ (50_000_000),
        .BAUD     (9600)
    ) u_imu_uart (
        .Clk   (loc_clk50m),
        .Rst_n (reset_n   ),
        .Rx    (imu_rx    ),
        .Data  (imu_byte  ),
        .Valid (imu_byte_v)
    );

    jy901_parse u_imu_parse (
        .Clk      (loc_clk50m),
        .Rst_n    (reset_n   ),
        .RxData   (imu_byte  ),
        .RxValid  (imu_byte_v),
        .Roll     (imu_roll  ),
        .Pitch    (imu_pitch ),
        .Yaw      (imu_yaw   ),
        .AngValid (imu_ang_v ),
        .LinkOk   (imu_link  ),
        .OkCnt    (imu_ok_cnt ),
        .ErrCnt   (imu_err_cnt),
        .DbgBytes (imu_dbg_bytes)
    );

    wire [7:0]  vis_rx_byte;
    wire        vis_rx_valid;
    wire [7:0]  vis_tx_byte;
    wire        vis_tx_valid;
    wire        vis_tx_busy;

    wire        vis_valid;
    wire [7:0]  vis_cls, vis_conf, vis_seq;
    wire [15:0] vis_cx, vis_cy, vis_w, vis_h;
    wire [15:0] vis_ping_cnt, vis_pong_cnt, vis_cnt, vis_err_cnt;
    wire        vis_link;

    wire        vis_mo_valid, vis_mo_rejected, vis_link_live;
    wire [7:0]  vis_mo_code, vis_mo_seq, vis_mo_lastcode;
    wire [15:0] vis_mo_arg0, vis_mo_arg1, vis_mo_arg2, vis_mo_gapms;

    wire        vis_start_t3, vis_start_t4;

    uart_rx_8n1 #(
        .CLK_FREQ (50_000_000),
        .BAUD     (115200)
    ) u_vis_uart_rx (
        .Clk   (loc_clk50m),
        .Rst_n (reset_n   ),
        .Rx    (uart1_rx  ),
        .Data  (vis_rx_byte),
        .Valid (vis_rx_valid)
    );

    vis_uart_bridge #(
        .CLK_FREQ        (50_000_000),
        .VIS_TIMEOUT_MS  (500),
        .LINK_TIMEOUT_MS (500),         
        .ALLOW_ROTATE    (0)            
    ) u_vis_bridge (
        .Clk       (loc_clk50m),
        .Rst_n     (reset_n   ),
        .RxData    (vis_rx_byte),
        .RxValid   (vis_rx_valid),
        .TxData    (vis_tx_byte),
        .TxValid   (vis_tx_valid),
        .TxBusy    (vis_tx_busy),
        .VisValid  (vis_valid),
        .VisCls    (vis_cls),
        .VisCx     (vis_cx),  .VisCy (vis_cy),
        .VisW      (vis_w),   .VisH  (vis_h),
        .VisConf   (vis_conf),.VisSeq(vis_seq),
        .PingCnt   (vis_ping_cnt), .PongCnt (vis_pong_cnt),
        .VisCnt    (vis_cnt), .LinkAlive (vis_link), .ErrCnt (vis_err_cnt),
        .OsdValid  (osd_valid), .OsdMode (osd_mode_cmd),    

        .MoValid     (vis_mo_valid),
        .MoCode      (vis_mo_code),
        .MoArg0      (vis_mo_arg0), .MoArg1 (vis_mo_arg1), .MoArg2 (vis_mo_arg2),
        .MoSeq       (vis_mo_seq),
        .MoRejected  (vis_mo_rejected),
        .MoLastCode  (vis_mo_lastcode),

        .MoLinkLive  (vis_link_live),
        .MoGapMs     (vis_mo_gapms),

        .StartT3     (vis_start_t3),
        .StartT4     (vis_start_t4)
    );

    uart_tx_8n1 #(
        .CLK_FREQ (50_000_000),
        .BAUD     (115200)
    ) u_vis_uart_tx (
        .Clk     (loc_clk50m),
        .Rst_n   (reset_n   ),
        .TxData  (vis_tx_byte),
        .TxValid (vis_tx_valid),
        .Tx      (uart1_tx  ),
        .Busy    (vis_tx_busy),
        .TxDone  ()
    );

    wire        follow_valid;
    wire [7:0]  follow_code;
    wire [15:0] follow_arg0;
    wire [3:0]  follow_state;
    wire        follow_target_present;

    vision_follow_ctrl u_follow (
        .Clk           (loc_clk50m),
        .Rst_n         (reset_n),
        .VisValid      (vis_valid),
        .VisCls        (vis_cls),
        .VisCx         (vis_cx),
        .VisCy         (vis_cy),
        .VisW          (vis_w),
        .VisH          (vis_h),
        .VisConf       (vis_conf),
        .FollowValid   (follow_valid),       
        .FollowCode    (follow_code),
        .FollowArg0    (follow_arg0),
        .FsmState      (follow_state),
        .TargetPresent (follow_target_present)
    );

    reg        vis_obs_valid;
    reg  [7:0] vis_obs_cls;
    reg [15:0] vis_obs_cx, vis_obs_cy, vis_obs_w, vis_obs_h;
    reg  [7:0] vis_obs_conf;
    reg [15:0] vis_obs_cnt;
    reg  [3:0] vis_obs_fsm;

    always @(posedge loc_clk50m or negedge reset_n)
    if(!reset_n) begin
        vis_obs_valid <= 1'b0;
        vis_obs_cls   <= 8'd0;
        vis_obs_cx    <= 16'd0; vis_obs_cy <= 16'd0;
        vis_obs_w     <= 16'd0; vis_obs_h  <= 16'd0;
        vis_obs_conf  <= 8'd0;
        vis_obs_cnt   <= 16'd0;
        vis_obs_fsm   <= 4'd0;
    end
    else begin
        vis_obs_valid <= vis_valid;
        vis_obs_cls   <= vis_cls;
        vis_obs_cx    <= vis_cx;  vis_obs_cy <= vis_cy;
        vis_obs_w     <= vis_w;   vis_obs_h  <= vis_h;
        vis_obs_conf  <= vis_conf;
        vis_obs_cnt   <= vis_cnt;
        vis_obs_fsm   <= follow_state;
    end

    wire [OSD_TXT_COLS*8*3-1:0] vis_txt_a, vis_txt_b;
    wire                        vis_bank;

    vis_osd_text #(
        .COLS (OSD_TXT_COLS)
    ) u_vis_osd_text (
        .Clk       (loc_clk50m),
        .Rst_n     (reset_n),
        .VisValid  (vis_obs_valid),
        .VisCls    (vis_obs_cls),
        .VisCx     (vis_obs_cx),
        .VisCy     (vis_obs_cy),
        .VisConf   (vis_obs_conf),
        .VisCnt    (vis_obs_cnt),
        .FsmState  (vis_obs_fsm),
        .TxtA      (vis_txt_a),
        .TxtB      (vis_txt_b),
        .BankSel   (vis_bank)
    );

    att_align u_att_align (
        .ImuRoll  (imu_roll  ),
        .ImuPitch (imu_pitch ),
        .ImuYaw   (imu_yaw   ),
        .AngRoll  (att_roll  ),
        .AngPitch (att_pitch ),
        .AngYaw   (att_yaw   )
    );

    angle_text_fmt #(
        .OSD_DBG_EN (IMU_OSD_DBG_EN),     
        .COLS       (OSD_TXT_COLS)        
    ) u_imu_text (
        .Clk      (loc_clk50m),
        .Rst_n    (reset_n   ),
        .AngValid (imu_ang_v ),
        .LinkOk   (imu_link  ),
        .RollRaw  (att_roll  ),       
        .PitchRaw (att_pitch ),
        .YawRaw   (att_yaw   ),
        .TxtA     (imu_txt_a ),
        .TxtB     (imu_txt_b ),
        .DbgOk    (imu_ok_cnt ),       
        .DbgErr   (imu_err_cnt),
        .DbgBytes (imu_dbg_bytes),     
        .BankSel  (imu_bank  )
    );

    reg [1:0] imu_bank_sync;
    always @(posedge clk_vga or negedge reset_n)
    if(!reset_n) imu_bank_sync <= 2'b00;
    else         imu_bank_sync <= {imu_bank_sync[0], imu_bank};

    function [7:0] t1_hex;
        input [3:0] n;
        begin t1_hex=(n<10) ? (8'd48+n) : (8'd55+n);end
    endfunction
    function [7:0] t1_ungray;
        input [7:0] g;
        integer i;
        begin t1_ungray[7]=g[7];for(i=6;i>=0;i=i-1) t1_ungray[i]=t1_ungray[i+1]^g[i];end
    endfunction
    function [103:0] t1_line;
        input [7:0] c0,c1,c2,v;
        begin t1_line={{6{8'h20}},t1_hex(v[3:0]),t1_hex(v[7:4]),8'h20,8'h3a,c2,c1,c0};end
    endfunction
    reg [7:0] t1_bs0,t1_bs1,t1_cs0,t1_cs1,t1_ds0,t1_ds1;
    reg [7:0] t1_bshow,t1_cshow,t1_dshow;
    always @(posedge clk_vga or negedge reset_n)
    if(!reset_n) begin
        t1_bs0<=0;t1_bs1<=0;t1_cs0<=0;t1_cs1<=0;t1_ds0<=0;t1_ds1<=0;
        t1_bshow<=0;t1_cshow<=0;t1_dshow<=0;
    end else if(TASK1_DIAG_EN) begin
        t1_bs0<=t1_bad_gray;t1_bs1<=t1_bs0;
        t1_cs0<=t1_commit_gray;t1_cs1<=t1_cs0;
        t1_ds0<=t1_drop_gray;t1_ds1<=t1_ds0;
        if(disp_frame_begin) begin
            t1_bshow<=t1_ungray(t1_bs1);t1_cshow<=t1_ungray(t1_cs1);t1_dshow<=t1_ungray(t1_ds1);
        end
    end
    wire [OSD_TXT_W-1:0] t1_text={{104{1'b0}},
        t1_line(8'h44,8'h52,8'h50,t1_dshow),
        t1_line(8'h43,8'h4f,8'h4d,t1_cshow),
        t1_line(8'h42,8'h41,8'h44,t1_bshow)};
    wire [OSD_TXT_W-1:0] osd_txt=TASK1_DIAG_EN ? t1_text : imu_txt_a;

    wire [23:0] disp_data_osd;
    osd_overlay #(
        .DUAL_TEXT (!TASK1_DIAG_EN),
        .ROWS (OSD_DISP_ROWS),       
        .COLS (OSD_TXT_COLS)         
    ) u_osd (
        .PixClk (clk_vga     ),
        .Rst_n  (reset_n     ),
        .Hcount (hcount      ),
        .Vcount (vcount      ),
        .Din    (disp_data_ov),
        .Txt    (osd_txt     ),
        .TxtAlt (imu_txt_b   ),
        .TxtBank(imu_bank_sync[1]),
        .Dout   (disp_data_osd)
    );

    reg [1:0] vis_bank_sync;
    always @(posedge clk_vga or negedge reset_n)
    if(!reset_n) vis_bank_sync <= 2'b00;
    else         vis_bank_sync <= {vis_bank_sync[0], vis_bank};

    wire [OSD_TXT_COLS*8*3-1:0] vis_txt = vis_bank_sync[1] ? vis_txt_b : vis_txt_a;

    wire [23:0] disp_data_vis;

    osd_overlay #(
        .DUAL_TEXT (1),
        .ROWS (3),                   
        .COLS (OSD_TXT_COLS),
        .X0   (12'd32),
        .Y0   (12'd152),
        .COL_FG (24'hFFFF00),        
        .COL_BG (24'h000000),
        .COL_BRD(24'hFFFF00)         
    ) u_vis_osd (
        .PixClk (clk_vga     ),
        .Rst_n  (reset_n     ),
        .Hcount (hcount      ),
        .Vcount (vcount      ),
        .Din    (disp_data_osd),     
        .Txt    (vis_txt_a   ),
        .TxtAlt (vis_txt_b   ),      
        .TxtBank(vis_bank_sync[1]),
        .Dout   (disp_data_vis)
    );

    wire       osd_valid;        
    wire [1:0] osd_mode_cmd;     

    reg [1:0] osd_mode_r;        

    wire [1:0] osd_mode_pix;
    osd_mode_cdc u_osd_mode_cdc(
        .SrcClk(loc_clk50m), .DstClk(clk_vga), .Rst_n(reset_n),
        .DesiredMode(osd_mode_r), .FrameBegin(disp_frame_begin),
        .DisplayMode(osd_mode_pix)
    );
    wire [23:0] disp_data_final = (osd_mode_pix == 2'd2) ? disp_data_vis :
                                  (osd_mode_pix == 2'd1) ? disp_data_osd :
                                                         disp_data_mk;

    parameter LED1_IMU_LINK = 1;

    assign led = {~g_rst_p,
                  LED1_IMU_LINK ? imu_link : camera_init_done,
                  init_calib_complete,
                  pll_lock};

    wire car_arst_n = reset_n & pll_locked;
    reg [1:0] car_rst_sync;
    always @(posedge loc_clk50m or negedge car_arst_n)
        if(!car_arst_n) car_rst_sync <= 2'b00;
        else            car_rst_sync <= {car_rst_sync[0], 1'b1};
    wire car_rst_n = car_rst_sync[1];

    always @(posedge loc_clk50m or negedge car_rst_n)
    if(!car_rst_n) osd_mode_r <= 2'd1;
    else if(osd_valid) osd_mode_r <= osd_mode_cmd;

    wire signed [31:0] yaw_d100_pr = att_yaw * 32'sd18000;    
    reg  signed [15:0] att_yaw_d100;
    always @(posedge loc_clk50m or negedge car_rst_n)
    if(!car_rst_n) att_yaw_d100 <= 16'sd0;
    else           att_yaw_d100 <= yaw_d100_pr >>> 15;        

    wire [7:0]  st_mode, st_dir, st_fault, st_nav_state, st_nav_wp, st_avoid_state;
    wire signed [15:0] st_dutyl, st_dutyr, st_vell, st_velr;
    wire signed [31:0] st_odoml, st_odomr;
    wire signed [15:0] st_navx, st_navy, st_navyaw, st_remain;
    wire [15:0] st_rangef, st_rangel, st_ranger;
    wire        st_avoid_near, st_avoid_active;
    wire [7:0]  st_seg1_state, st_seg1_err;
    wire        st_seg1_done;
    wire signed [31:0] st_seg1_dist;

    wire        cmd_valid;
    wire [7:0]  cmd_code;
    wire [15:0] cmd_arg0, cmd_arg1, cmd_arg2;

    localparam STAGE1_CALIB = 0;       

    localparam AVOID_EN_SYS = 1;       

    localparam NAV_EN_SYS  = 0;         
    localparam SEG1_EN_SYS = 1;         

    wire        g_cmd_valid;
    wire [7:0]  g_cmd_code;
    wire [15:0] g_cmd_arg0, g_cmd_arg1, g_cmd_arg2;
    wire        gate_force_stop;
    wire [3:0]  gate_stop_reason;
    wire [15:0] gate_fresh_ms;
    wire        gate_last_uart;

    car_ctrl_top #(
        .AVOID_EN      (AVOID_EN_SYS),       
        .NAV_EN        (NAV_EN_SYS),         
        .SEG1_EN       (SEG1_EN_SYS),

        .SEG1_CRUISE_SPEED (8'd50),       
        .LOOP_EN_SEL   (STAGE1_CALIB),       
        .DBG_ENCDIR_EN (STAGE1_CALIB)        
    ) u_car_ctrl (
        .Clk      (loc_clk50m),
        .Rst_n    (car_rst_n),

        .CmdValid (g_cmd_valid), .CmdCode (g_cmd_code),
        .CmdArg0  (g_cmd_arg0),  .CmdArg1 (g_cmd_arg1), .CmdArg2 (g_cmd_arg2),

        .AngRoll  (att_roll), .AngPitch (att_pitch), .AngYaw (att_yaw_d100),

        .AngValid (imu_ang_v), .ImuLink (imu_link), .LinkAlive (vis_link_live),
        .EncA0 (enc_a0), .EncB0 (enc_b0), .EncA1 (enc_a1), .EncB1 (enc_b1),

        .UsEcho0 (us_echo0), .UsEcho1 (us_echo1), .UsEcho2 (1'b0),
        .Key (key), .Dip (STAGE1_CALIB ? 8'hFE : 8'hFF),    
        .MotorPwm (motor_pwm), .MotorDir (motor_dir), .MotorDirN (motor_dirn),
        .UsTrig0 (us_trig0), .UsTrig1 (us_trig1), .UsTrig2 (),
        .Beep (beep), .BLed (bled),
        .StMode (st_mode), .StDir (st_dir),
        .StDutyL (st_dutyl), .StDutyR (st_dutyr),
        .StVelL (st_vell), .StVelR (st_velr),
        .StOdomL (st_odoml), .StOdomR (st_odomr),
        .StRangeF (st_rangef), .StRangeL (st_rangel), .StRangeR (st_ranger),
        .StFault (st_fault),
        .StNavState (st_nav_state), .StNavWp (st_nav_wp),
        .StNavX (st_navx), .StNavY (st_navy), .StNavYaw (st_navyaw),
        .StRemainCm (st_remain),
        .StAvoidState (st_avoid_state), .StAvoidNear (st_avoid_near),
        .StAvoidActive (st_avoid_active),
         .StSeg1State (st_seg1_state), .StSeg1Done (st_seg1_done),
         .StSeg1DistMm (st_seg1_dist), .StSeg1Err (st_seg1_err),

        .SafetyStop (gate_force_stop)
    );

    reg [24:0] seq_cnt;
    reg [2:0] seq;
    reg [7:0] want_dir, dir_sent;
    reg cmd_valid_r;
    reg [7:0] cmd_code_r;
    reg [15:0] cmd_arg0_r;
    reg [15:0] cmd_arg1_r;

    reg [1:0] key_s1_sync, key_s2_sync, key_s3_sync, key_s4_sync;
    reg [15:0] key_sample_cnt;
    wire key_sample_tick = (key_sample_cnt == 16'd49_999);
    reg [4:0] key_s2_db_cnt, key_s3_db_cnt, key_s4_db_cnt;
    reg key_s2_n_stable, key_s3_n_stable, key_s4_n_stable;
    reg key_s2_n_prev, key_s3_n_prev, key_s4_n_prev;
    wire s4_release = ~key_s4_n_prev & key_s4_n_stable;    

    wire s2_release = ~key_s2_n_prev & key_s2_n_stable;    
    wire s3_release = ~key_s3_n_prev & key_s3_n_stable;    

    reg [2:0] gear_sel;
    wire [7:0] gear_dir = (gear_sel == 3'd1) ? 8'h03 :    
                          (gear_sel == 3'd2) ? 8'h04 :    
                          (gear_sel == 3'd3) ? 8'h05 :    
                          (gear_sel == 3'd4) ? 8'h06 :    
                                               8'h00;

    wire [7:0] gear_spd = ((gear_sel == 3'd3) || (gear_sel == 3'd4)) ? 8'd35 : 8'd60;

    wire [7:0] dir_out = (gear_sel != 3'd0) ? gear_dir : want_dir;
    wire [7:0] spd_out = (gear_sel != 3'd0) ? gear_spd : 8'd60;

    reg [15:0] s2_hold_cnt;
    reg        s2_long_done;
    wire s2_long_evt = (s2_hold_cnt == 16'd1000) & ~s2_long_done;

    reg [15:0] s3_hold_cnt;
    reg        s3_long_done;
    wire s3_long_evt = (s3_hold_cnt == 16'd1000) & ~s3_long_done;

    reg        seg1_abort_pending;

    wire t3_start_evt = s2_long_evt | (vis_start_t3 & (st_mode != `MODE_AVOID));
    wire t3_want_on   = s2_long_evt ? ~(st_mode == `MODE_AVOID) : 1'b1;
    wire [7:0] t4_mode_v = (SEG1_EN_SYS != 0) ? `MODE_SEG1 : `MODE_NAV;
    wire t4_start_evt = s3_long_evt | (vis_start_t4 & (st_mode != t4_mode_v));
    reg rq_mode_seen;  

    localparam integer ROUTE_N = 4;
    localparam signed [15:0] WP0_X = 16'sd100, WP0_Y = 16'sd0;
    localparam signed [15:0] WP1_X = 16'sd100, WP1_Y = 16'sd100;
    localparam signed [15:0] WP2_X = 16'sd0,   WP2_Y = 16'sd100;
    localparam signed [15:0] WP3_X = 16'sd0,   WP3_Y = 16'sd0;

    localparam [2:0] RQ_IDLE = 3'd0, RQ_MODE = 3'd1, RQ_WP = 3'd2,
                     RQ_START= 3'd3, RQ_RUN  = 3'd4;
    localparam [7:0] NAV_ARRIVED_V = 8'h04;     

    reg [2:0] rq;          
    reg [2:0] rt_idx;      

    wire signed [15:0] rt_x = (rt_idx == 3'd0) ? WP0_X :
                              (rt_idx == 3'd1) ? WP1_X :
                              (rt_idx == 3'd2) ? WP2_X : WP3_X;
    wire signed [15:0] rt_y = (rt_idx == 3'd0) ? WP0_Y :
                              (rt_idx == 3'd1) ? WP1_Y :
                              (rt_idx == 3'd2) ? WP2_Y : WP3_Y;

    wire nav_ui_on = (rq != RQ_IDLE);       

    wire seg1_ui_on = (SEG1_EN_SYS != 0) && (st_mode == `MODE_SEG1) &&
                    (nav_ui_on || (st_seg1_state != 8'd0));
    wire seg1_exit_evt = seg1_ui_on &&
                         (s3_long_evt || (s2_release && !s2_long_done) ||
                          (s3_release && !s3_long_done) || ~key_s1_sync[1]);

    wire nav_exit_evt = nav_ui_on & ( s3_long_evt
                                    | (s2_release && !s2_long_done)
                                    | (s3_release && !s3_long_done)
                                    | ~key_s1_sync[1] );

    always @(posedge loc_clk50m or negedge car_rst_n)
    if(!car_rst_n) begin
        seq <= 3'd0; seq_cnt <= 25'd0;
        want_dir <= 8'h00; dir_sent <= 8'hFF;
        cmd_valid_r <= 1'b0; cmd_code_r <= 8'h00; cmd_arg0_r <= 16'h0000;
        cmd_arg1_r <= 16'h0000;
        key_s1_sync <= 2'b11; key_s2_sync <= 2'b11; key_s3_sync <= 2'b11; key_s4_sync <= 2'b11;
        key_sample_cnt <= 16'd0;
        key_s2_db_cnt <= 5'd0; key_s3_db_cnt <= 5'd0; key_s4_db_cnt <= 5'd0;
        key_s2_n_stable <= 1'b1; key_s3_n_stable <= 1'b1; key_s4_n_stable <= 1'b1;
        key_s2_n_prev <= 1'b1; key_s3_n_prev <= 1'b1; key_s4_n_prev <= 1'b1;
        s2_hold_cnt <= 16'd0; s2_long_done <= 1'b0;
        gear_sel <= 3'd0;
        s3_hold_cnt <= 16'd0; s3_long_done <= 1'b0;
        seg1_abort_pending <= 1'b0;

    end else begin
        key_s1_sync <= {key_s1_sync[0], key[0]};
        key_s2_sync <= {key_s2_sync[0], key[1]};
        key_s3_sync <= {key_s3_sync[0], key[2]};
        key_s4_sync <= {key_s4_sync[0], key[3]};
        key_s2_n_prev <= key_s2_n_stable;
        key_s3_n_prev <= key_s3_n_stable;
        key_s4_n_prev <= key_s4_n_stable;

        if(key_sample_tick) key_sample_cnt <= 16'd0;
        else                key_sample_cnt <= key_sample_cnt + 1'b1;

        if(key_sample_tick) begin
            if(key_s2_sync[1] == key_s2_n_stable) key_s2_db_cnt <= 5'd0;
            else if(key_s2_db_cnt >= 5'd19) begin
                key_s2_n_stable <= key_s2_sync[1];
                key_s2_db_cnt <= 5'd0;
            end else key_s2_db_cnt <= key_s2_db_cnt + 1'b1;

            if(key_s3_sync[1] == key_s3_n_stable) key_s3_db_cnt <= 5'd0;
            else if(key_s3_db_cnt >= 5'd19) begin
                key_s3_n_stable <= key_s3_sync[1];
                key_s3_db_cnt <= 5'd0;
            end else key_s3_db_cnt <= key_s3_db_cnt + 1'b1;

            if(key_s4_sync[1] == key_s4_n_stable) key_s4_db_cnt <= 5'd0;
            else if(key_s4_db_cnt >= 5'd19) begin
                key_s4_n_stable <= key_s4_sync[1];
                key_s4_db_cnt <= 5'd0;
            end else key_s4_db_cnt <= key_s4_db_cnt + 1'b1;
        end

        if(key_sample_tick) begin
            if(key_s2_n_stable == 1'b1) s2_hold_cnt <= 16'd0;
            else if(s2_hold_cnt != 16'hFFFF) s2_hold_cnt <= s2_hold_cnt + 16'd1;
        end
        if(key_s2_n_stable == 1'b1) s2_long_done <= 1'b0;
        else if(s2_long_evt)        s2_long_done <= 1'b1;

        if(key_sample_tick) begin
            if(key_s3_n_stable == 1'b1) s3_hold_cnt <= 16'd0;
            else if(s3_hold_cnt != 16'hFFFF) s3_hold_cnt <= s3_hold_cnt + 16'd1;
        end
        if(key_s3_n_stable == 1'b1) s3_long_done <= 1'b0;
        else if(s3_long_evt)        s3_long_done <= 1'b1;

        cmd_valid_r <= 1'b0;
        if(seq_cnt < 25'd14_999_999) seq_cnt <= seq_cnt + 1'b1;
        else case(seq)
            3'd0: begin cmd_valid_r<=1'b1; cmd_code_r<=8'h03;
                        cmd_arg0_r<=16'h0000; cmd_arg1_r<=16'h0000; seq<=3'd1; end
            3'd1: begin cmd_valid_r<=1'b1; cmd_code_r<=8'h01;
                        cmd_arg0_r<={8'd60, 8'h00}; cmd_arg1_r<=16'h0000;
                        dir_sent<=8'h00; seq<=3'd2; end
            default: begin

                if(nav_exit_evt) begin
                    want_dir <= 8'h00;
                    gear_sel <= 3'd0;
                end

                else if(t3_start_evt) begin
                    want_dir <= t3_want_on ? 8'h01 : 8'h00;
                    gear_sel <= 3'd0;                         
                end

                else if(!key_s1_sync[1]) begin
                    want_dir <= 8'h00; gear_sel <= 3'd0;
                end

                else if(s2_release && !s2_long_done) begin
                    gear_sel <= 3'd0;
                    want_dir <= (want_dir==8'h01) ? 8'h00 : 8'h01;
                end

                else if(s3_release && !s3_long_done) begin
                    gear_sel <= 3'd0;
                    want_dir <= (want_dir==8'h02) ? 8'h00 : 8'h02;
                end

                else if(s4_release) begin
                    gear_sel <= (gear_sel == 3'd4) ? 3'd0 : (gear_sel + 3'd1);
                end

                if(seg1_abort_pending) begin

                    if(key_s1_sync[1]) begin
                        cmd_valid_r  <= 1'b1; cmd_code_r <= `CMD_MODE;
                        cmd_arg0_r   <= {8'd0, `MODE_MANUAL}; cmd_arg1_r <= 16'h0000;
                        seg1_abort_pending <= 1'b0;
                    end
                end
                else if(seg1_exit_evt) begin
                    cmd_valid_r  <= 1'b1; cmd_code_r <= `CMD_SEG1_ABORT;
                    cmd_arg0_r   <= 16'h0000; cmd_arg1_r <= 16'h0000;
                    seg1_abort_pending <= 1'b1;
                end
                else if(t3_start_evt) begin

                    cmd_valid_r  <= 1'b1; cmd_code_r <= 8'h03;                  
                    cmd_arg0_r   <= {8'd0, t3_want_on ? `MODE_AVOID : `MODE_MANUAL};
                    cmd_arg1_r   <= 16'h0000;
                end

                else if(nav_exit_evt) begin
                    cmd_valid_r  <= 1'b1; cmd_code_r <= 8'h03;                  
                    cmd_arg0_r   <= {8'd0, 8'h00};                              
                    cmd_arg1_r   <= 16'h0000;
                end

                else if(rq == RQ_MODE) begin
                    cmd_valid_r  <= 1'b1; cmd_code_r <= 8'h03;                  
                     cmd_arg0_r   <= {8'd0, (SEG1_EN_SYS != 0) ? `MODE_SEG1 : `MODE_NAV};
                    cmd_arg1_r   <= 16'h0000;
                end
                else if((rq == RQ_WP) && (SEG1_EN_SYS == 0)) begin
                    cmd_valid_r  <= 1'b1; cmd_code_r <= 8'h10;                  
                    cmd_arg0_r   <= rt_x;                                       
                    cmd_arg1_r   <= rt_y;                                       
                end
                else if(rq == RQ_START) begin
                    cmd_valid_r  <= 1'b1;
                    cmd_code_r  <= (SEG1_EN_SYS != 0) ? `CMD_SEG1_START : `CMD_NAV_START;
                    cmd_arg0_r   <= 16'h0000; cmd_arg1_r <= 16'h0000;
                end
                else if(dir_out != dir_sent) begin
                    cmd_valid_r<=1'b1; cmd_code_r<=8'h01;                       
                    cmd_arg0_r<={spd_out, dir_out}; cmd_arg1_r<=16'h0000;
                    dir_sent<=dir_out;
                end
            end
        endcase
    end

    always @(posedge loc_clk50m or negedge car_rst_n)
    if(!car_rst_n) begin
        rq <= RQ_IDLE; rt_idx <= 3'd0; rq_mode_seen <= 1'b0;
    end
    else begin
        if(rq == RQ_IDLE) rq_mode_seen <= 1'b0;
        else if(st_mode == t4_mode_v) rq_mode_seen <= 1'b1;
        case(rq)
        RQ_IDLE : if(t4_start_evt) begin rt_idx <= 3'd0; rq <= RQ_MODE; end
        RQ_MODE : rq <= (SEG1_EN_SYS != 0) ? RQ_START : RQ_WP;   
        RQ_WP   : rq <= RQ_START;      
        RQ_START: rq <= RQ_RUN;        
        RQ_RUN  : begin
            if(nav_exit_evt || t3_start_evt)
                rq <= RQ_IDLE;                                   
            else if(t4_start_evt) begin

                rq <= RQ_MODE; rt_idx <= 3'd0; rq_mode_seen <= 1'b0;
            end
            else if((st_mode == `MODE_MANUAL) && rq_mode_seen)
                rq <= RQ_IDLE;                                   
            else if((SEG1_EN_SYS == 0) && (st_nav_state == NAV_ARRIVED_V) && (rt_idx < (ROUTE_N-1))) begin
                rt_idx <= rt_idx + 3'd1;
                rq     <= RQ_WP;                                 
            end
        end
        default : rq <= RQ_IDLE;
        endcase
    end

    assign cmd_valid = cmd_valid_r;
    assign cmd_code  = cmd_code_r;
    assign cmd_arg0  = cmd_arg0_r;
    assign cmd_arg1  = cmd_arg1_r;
    assign cmd_arg2  = 16'd0;

    motion_safety_gate #(
        .CLK_FREQ          (50_000_000),
        .FRESH_TIMEOUT_MS  (500),       
        .STICKY_LOCAL      (1),         
        .REQ_LINK_ALIVE    (0),         
        .LINK_STOP_ON_UART (1)          
    ) u_safety_gate (
        .Clk       (loc_clk50m),
        .Rst_n     (car_rst_n),
        .LocValid  (cmd_valid),  .LocCode (cmd_code),
        .LocArg0   (cmd_arg0),   .LocArg1 (cmd_arg1),  .LocArg2 (cmd_arg2),
        .UartValid (vis_mo_valid), .UartCode (vis_mo_code),
        .UartArg0  (vis_mo_arg0),  .UartArg1 (vis_mo_arg1), .UartArg2 (vis_mo_arg2),
        .S1_Estop  (~key_s1_sync[1]),           
        .LinkLive  (vis_link_live),
        .OutValid  (g_cmd_valid), .OutCode (g_cmd_code),
        .OutArg0   (g_cmd_arg0),  .OutArg1 (g_cmd_arg1), .OutArg2 (g_cmd_arg2),
        .ForceStop (gate_force_stop), .StopReason (gate_stop_reason),
        .FreshMs   (gate_fresh_ms),   .LastSrcUart (gate_last_uart)
    );

    wire [15:0] smg_vel_l = st_vell[15] ? (~st_vell[15:0] + 16'd1) : st_vell[15:0];
    wire [15:0] smg_vel_r = st_velr[15] ? (~st_velr[15:0] + 16'd1) : st_velr[15:0];

    function [11:0] bcd3;
        input [15:0] v;
        reg   [27:0] sh;
        integer      i;
        begin
            sh = {12'd0, (v > 16'd999) ? 16'd999 : v};
            for(i = 0; i < 16; i = i + 1) begin
                if(sh[27:24] >= 4'd5) sh[27:24] = sh[27:24] + 4'd3;
                if(sh[23:20] >= 4'd5) sh[23:20] = sh[23:20] + 4'd3;
                if(sh[19:16] >= 4'd5) sh[19:16] = sh[19:16] + 4'd3;
                sh = sh << 1;
            end
            bcd3 = {sh[27:24], sh[23:20], sh[19:16]};
        end
    endfunction

    wire [15:0] nav_x_abs = st_navx[15] ? (~st_navx + 16'd1) : st_navx;
    wire [15:0] nav_y_abs = st_navy[15] ? (~st_navy + 16'd1) : st_navy;

    reg [31:0] smg_data;
    always @(posedge loc_clk50m or negedge car_rst_n)
    if(!car_rst_n) smg_data <= {4'hF, 12'd0, 4'hF, 12'd0};
    else if(nav_ui_on)                       
        smg_data <= { (st_navx[15] ? 4'hE : 4'hF), bcd3(nav_x_abs),    
                      (st_navy[15] ? 4'hE : 4'hF), bcd3(nav_y_abs) };  
    else                                     
        smg_data <= { 4'hF,              
                      bcd3(smg_vel_l),   
                      4'hF,              
                      bcd3(smg_vel_r) }; 

    wire [7:0] smg_sel;
    wire [6:0] smg_seg;

    hex8 u_hex8 (
        .clk      (loc_clk50m),
        .reset_n  (car_rst_n),
        .en       (1'b1),
        .disp_data(smg_data),
        .sel      (smg_sel),
        .seg      (smg_seg)
    );

    hc595_driver u_hc595 (
        .clk     (loc_clk50m),
        .reset_n (car_rst_n),
        .data    ({1'b1, smg_seg, smg_sel}),    
        .s_en    (1'b1),
        .sh_cp   (seg_shcp),
        .st_cp   (seg_stcp),
        .ds      (seg_ds)
    );

    wire [7:0]         tlm_state = st_seg1_state;
    wire [7:0]         tlm_err   = st_seg1_err | 8'h40;    
    wire signed [15:0] tlm_vell  = st_vell;
    wire signed [15:0] tlm_velr  = st_velr;
    wire [15:0]        tlm_rng   = st_rangef;
    wire signed [15:0] tlm_yaw   = att_yaw_d100;
    wire [15:0]        tlm_leg2  = st_seg1_dist[15:0];

    reg [7:0] dbg_last_code;
    reg [3:0] dbg_start_cnt;
    always @(posedge loc_clk50m or negedge car_rst_n)
    if(!car_rst_n) begin
        dbg_last_code <= 8'h00;
        dbg_start_cnt <= 4'd0;
    end
    else if(g_cmd_valid) begin
        dbg_last_code <= g_cmd_code;
        if(g_cmd_code == `CMD_SEG1_START) dbg_start_cnt <= dbg_start_cnt + 4'd1;
    end

    wire        tlm_s1_now = ~key_s1_sync[1];       
    wire [15:0] tlm_diag1  = { st_mode[7:0], gate_stop_reason[3:0],
                               gate_force_stop, tlm_s1_now, 2'b00 };
    wire [15:0] tlm_diag2  = { dbg_start_cnt, dbg_last_code[7:0], rq[2:0], 1'b0 };

    localparam [5:0] TLM_CR = 6'd41, TLM_LF = 6'd42;

    reg [23:0]  tlm_cnt;
    wire        tlm_tick = (tlm_cnt == 24'd10_000_000);    
    reg         tlm_run;
    reg [5:0]   tlm_step;
    reg [159:0] tlm_sh;
    reg         tlm_txv;
    reg [7:0]   tlm_txd;
    wire        tlm_busy;

    wire [3:0]  tlm_nib = tlm_sh[159:156];
    wire [7:0]  tlm_asc = (tlm_nib < 4'd10) ? (8'h30 + {4'd0, tlm_nib})
                                            : (8'h37 + {4'd0, tlm_nib});

    always @(posedge loc_clk50m or negedge car_rst_n)
    if(!car_rst_n) begin
        tlm_cnt <= 24'd0; tlm_run <= 1'b0; tlm_step <= 6'd0;
        tlm_sh  <= 160'd0; tlm_txv <= 1'b0; tlm_txd <= 8'd0;
    end else begin
        tlm_cnt <= tlm_tick ? 24'd0 : (tlm_cnt + 24'd1);
        tlm_txv <= 1'b0;
        if(tlm_tick && !tlm_run) begin
            tlm_sh <= {
                8'hA5, 8'h5A, 8'h11, 8'h22,
                tlm_state, tlm_err,
                tlm_vell[7:0],  tlm_vell[15:8],
                tlm_velr[7:0],  tlm_velr[15:8],
                tlm_rng[7:0],   tlm_rng[15:8],
                tlm_yaw[7:0],   tlm_yaw[15:8],
                tlm_leg2[7:0],  tlm_leg2[15:8],
                tlm_diag1[7:0], tlm_diag1[15:8],
                tlm_diag2[7:0], tlm_diag2[15:8]
            };
            tlm_run <= 1'b1; tlm_step <= 6'd0;
        end
        else if(tlm_run && !tlm_busy && !tlm_txv) begin
            case(tlm_step)
            6'd0   : begin tlm_txd <= 8'h23; tlm_txv <= 1'b1; tlm_step <= 6'd1; end
            TLM_CR : begin tlm_txd <= 8'h0D; tlm_txv <= 1'b1; tlm_step <= TLM_LF; end
            TLM_LF : begin tlm_txd <= 8'h0A; tlm_txv <= 1'b1; tlm_step <= 6'd0;
                           tlm_run <= 1'b0; end
            default: begin
                tlm_txd  <= tlm_asc; tlm_txv <= 1'b1;
                tlm_sh   <= {tlm_sh[155:0], 4'd0};
                tlm_step <= tlm_step + 6'd1;
            end
            endcase
        end
    end

    uart_tx_8n1 #(
        .CLK_FREQ (50_000_000),
        .BAUD     (115200)
    ) u_tlm (
        .Clk     (loc_clk50m),
        .Rst_n   (car_rst_n),
        .TxData  (tlm_txd),
        .TxValid (tlm_txv),
        .Tx      (uart_tx),
        .Busy    (tlm_busy),
        .TxDone  ()
    );

    disp_driver disp_driver(
		.ClkDisp(clk_vga),
		.Rst_n(reset_n),
		.Data(disp_data_final),
		.DataReq(rdfifo_rden),
		.H_Addr(hcount),
		.V_Addr(vcount),
		.Disp_HS(video_hs),
		.Disp_VS(video_vs),
		.Disp_Red(video_r),
		.Disp_Green(video_g),
		.Disp_Blue(video_b),
        .Frame_Begin(disp_frame_begin),
		.Disp_DE(video_de),
		.Disp_PCLK()
	);

	dvi_encoder u_dvi_encoder(
		.pixelclk(clk_vga), 
		.pixelclk5x(clk_vgax5), 
		.rst_n(reset_n), 
		.blue_din(video_b), 
		.green_din(video_g), 
		.red_din(video_r), 
		.hsync(video_hs), 
		.vsync(video_vs), 
		.de(video_de), 
		.tmds_clk_p(tmds_clk_p),
		.tmds_clk_n(tmds_clk_n),
		.tmds_data_p(tmds_data_p), 
		.tmds_data_n(tmds_data_n)  
	);

endmodule

