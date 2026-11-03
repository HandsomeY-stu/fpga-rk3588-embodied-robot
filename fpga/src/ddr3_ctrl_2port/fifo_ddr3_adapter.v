/*
 * fifo_ddr3_adapter.v
 * FIFO侧与DDR3突发访问侧之间的数据/握手适配。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
module fifo_ddr3_adapter #(
    parameter integer LINE_WORDS=1280, FRAME_ROWS=720, ROW_FLIP_EN=1,
    parameter integer DIAG_EN=0, FIFO_RESET_CYCLES=32
)(
    input ui_clk, rst_n, init_calib_complete, app_rdy, app_wdf_rdy,
    input app_rd_data_valid, input [127:0] app_rd_data,
    output [27:0] app_addr, output app_en, app_wdf_wren, app_wdf_end,
    output [2:0] app_cmd, output [127:0] app_wdf_data,
    input rd_load, wr_load, wr_frame_good,

    input [27:0] app_addr_rd_min, app_addr_rd_max, app_addr_wr_min, app_addr_wr_max,
    input [7:0] rd_bust_len, wr_bust_len,
    input wr_clk, wfifo_wren, input [15:0] wfifo_din,
    output wrfifo_full, input rd_clk, rfifo_rden,
    output rdfifo_empty, output [15:0] rfifo_dout,
    output video_ready,
    output [7:0] diag_bad_gray, diag_commit_gray, diag_drop_gray
);
    localparam integer F=LINE_WORDS*FRAME_ROWS;
    localparam integer A=$clog2(3*F+1);
    localparam integer CW=$clog2(FIFO_RESET_CYCLES+1);
    localparam [2:0] W_RESET=0,W_ARM=1,W_RUN=2,W_ALLOC=3;
    localparam [1:0] R_WAIT=0,R_DRAIN=1,R_RESET=2,R_RUN=3;
    function [A-1:0] base;
        input [1:0] id;
        begin case(id) 0:base=0; 1:base=F; default:base=2*F; endcase end
    endfunction
    function [A-1:0] wlast;
        input [1:0] id;
        begin case(id) 0:wlast=F-8; 1:wlast=2*F-8; default:wlast=3*F-8; endcase end
    endfunction
    function [A-1:0] rfirst;
        input [1:0] id;
        begin rfirst=base(id)+(ROW_FLIP_EN ? (FRAME_ROWS-1)*LINE_WORDS : 0); end
    endfunction
    function [A-1:0] rlast;
        input [1:0] id;
        begin rlast=base(id)+(ROW_FLIP_EN ? LINE_WORDS-8 : F-8); end
    endfunction
    reg [1:0] write_id, read_id, ready_id;
    reg ready_valid, read_valid;
    reg [2:0] ws;
    reg [1:0] rs;
    reg [CW-1:0] wc,rc;
    reg [A-1:0] wa,ra;
    reg [7:0] read_col;
    reg write_done, eof_seen, eof_good, read_done;

    wire grant=(ws==W_ARM);

    reg [1:0] grant_sync;
    reg wr_load_d,capture_on,capture_bad,start_toggle,end_toggle,end_good;
    wire fifo_reset=(ws==W_RESET) || !rst_n;
    always @(posedge wr_clk or negedge rst_n)
    if(!rst_n) begin
        grant_sync<=0; wr_load_d<=0; capture_on<=0; capture_bad<=0;
        start_toggle<=0; end_toggle<=0; end_good<=0;
    end else begin
        grant_sync<={grant_sync[0],grant}; wr_load_d<=wr_load;
        if(wr_load && !wr_load_d) begin
            capture_on<=grant_sync[1]; capture_bad<=0;
            if(grant_sync[1]) start_toggle<=~start_toggle;
        end
        if(capture_on && wfifo_wren && wrfifo_full) capture_bad<=1;
        if(!wr_load && wr_load_d) begin
            end_good<=wr_frame_good && !capture_bad;
            if(capture_on) end_toggle<=~end_toggle;
            capture_on<=0;
        end
    end
    reg [2:0] start_sync,end_sync,vs_sync;
    wire start_event=start_sync[2]^start_sync[1];
    wire end_event=end_sync[2]^end_sync[1];
    wire display_blank=vs_sync[2] && !vs_sync[1];
    always @(posedge ui_clk or negedge rst_n)
    if(!rst_n) begin start_sync<=0;end_sync<=0;vs_sync<=0; end
    else begin
        start_sync<={start_sync[1:0],start_toggle};
        end_sync<={end_sync[1:0],end_toggle};
        vs_sync<={vs_sync[1:0],rd_load};
    end

    wire [9:0] wfifo_rcount,rfifo_wcount;
    wire [127:0] wfifo_q;
    reg [8:0] outstanding;

    wire [10:0] reserved={1'b0,rfifo_wcount}+{2'b0,outstanding};
    wire write_fire;

    wire wfifo_empty;
    reg [7:0] write_left;
    always @(posedge ui_clk or negedge rst_n)
    if(!rst_n) write_left<=0;
    else if(ws!=W_RUN || write_done) write_left<=0;
    else if(write_left==0) begin
        if(wfifo_rcount>=LINE_WORDS/8) write_left<=LINE_WORDS/8;
        else if(eof_seen && eof_good && !wfifo_empty)
            write_left<=1;  
    end else if(write_fire) write_left<=write_left-1'b1;
    wire want_write=(ws==W_RUN) && !write_done && (write_left!=0) && !wfifo_empty;
    assign write_fire=init_calib_complete && want_write && app_rdy && app_wdf_rdy;
    wire read_fire=init_calib_complete && (rs==R_RUN) && read_valid && !read_done
                   && !display_blank && (reserved<11'd448) && app_rdy && !write_fire;
    wire commit=(ws==W_RUN) && eof_seen && eof_good && write_done;
    wire consume_ready=(rs==R_RESET) && (rc==1) && ready_valid;
    assign video_ready=read_valid;
    assign app_en=write_fire || read_fire;
    assign app_wdf_wren=write_fire;
    assign app_wdf_end=write_fire;
    assign app_cmd=write_fire ? 3'd0 : 3'd1;
    assign app_addr=write_fire ? {{(28-A){1'b0}},wa} : {{(28-A){1'b0}},ra};
    assign app_wdf_data=wfifo_q;

    always @(posedge ui_clk or negedge rst_n)
    if(!rst_n) begin
        write_id<=0;read_id<=1;ready_id<=0;ready_valid<=0;read_valid<=0;
        ws<=W_RESET;rs<=R_WAIT;wc<=FIFO_RESET_CYCLES;rc<=0;
        wa<=0;ra<=0;read_col<=0;outstanding<=0;
        write_done<=0;eof_seen<=0;eof_good<=0;read_done<=1;
    end else begin
        case({read_fire,app_rd_data_valid})
          2'b10:outstanding<=outstanding+1'b1;
          2'b01:outstanding<=outstanding-1'b1;
          default:outstanding<=outstanding;
        endcase
        case(ws)
          W_ALLOC:begin

            if(read_id!=0 && (!ready_valid || ready_id!=0)) write_id<=0;
            else if(read_id!=1 && (!ready_valid || ready_id!=1)) write_id<=1;
            else write_id<=2;
            wc<=FIFO_RESET_CYCLES;ws<=W_RESET;
          end
          W_RESET:begin
            if(wc!=0) wc<=wc-1'b1;
            else begin
                wa<=base(write_id);write_done<=0;eof_seen<=0;eof_good<=0;
                ws<=W_ARM;
            end
          end
          W_ARM:if(start_event) ws<=W_RUN;
          W_RUN:begin
            if(end_event) begin eof_seen<=1;eof_good<=end_good;end
            if(write_fire) begin
                if(wa==wlast(write_id)) write_done<=1;
                else wa<=wa+8;
            end
            if(eof_seen && !eof_good) ws<=W_ALLOC;
            if(commit) begin ready_id<=write_id;ready_valid<=1;ws<=W_ALLOC;end
          end
          default:ws<=W_ALLOC;
        endcase

        if(display_blank) rs<=R_DRAIN;
        else case(rs)
          R_DRAIN:if(outstanding==0) begin rs<=R_RESET;rc<=FIFO_RESET_CYCLES;end
          R_RESET:begin
            if(rc>1) rc<=rc-1'b1;
            else begin
                if(ready_valid) begin
                    read_id<=ready_id;read_valid<=1;
                    if(!commit) ready_valid<=0;
                    ra<=rfirst(ready_id);
                end else ra<=rfirst(read_id);
                read_col<=0;read_done<=0;rs<=R_RUN;
            end
          end
          R_RUN:if(read_fire) begin
            if(ra==rlast(read_id)) read_done<=1;
            else if(ROW_FLIP_EN && read_col==LINE_WORDS/8-1)
                ra<=ra-(2*LINE_WORDS-8);
            else ra<=ra+8;
            if(read_col==LINE_WORDS/8-1) read_col<=0;
            else read_col<=read_col+1'b1;
          end
          default:rs<=rs;
        endcase
    end

    reg [7:0] bad_count,commit_count,drop_count;
    reg [2:0] cam_level_sync;
    always @(posedge ui_clk or negedge rst_n)
    if(!rst_n) begin bad_count<=0;commit_count<=0;drop_count<=0;cam_level_sync<=0;end
    else if(DIAG_EN) begin
        cam_level_sync<={cam_level_sync[1:0],wr_load};
        if(end_event && !end_good) bad_count<=bad_count+1'b1;
        if(commit) commit_count<=commit_count+1'b1;
        if(cam_level_sync[1] && !cam_level_sync[2] && ws!=W_ARM)
            drop_count<=drop_count+1'b1;
    end
    reg [7:0] bad_gray,commit_gray,drop_gray;
    always @(posedge ui_clk or negedge rst_n)
    if(!rst_n) begin bad_gray<=0;commit_gray<=0;drop_gray<=0;end
    else if(DIAG_EN) begin
        bad_gray<=bad_count^(bad_count>>1);
        commit_gray<=commit_count^(commit_count>>1);
        drop_gray<=drop_count^(drop_count>>1);
    end
    assign diag_bad_gray=DIAG_EN ? bad_gray : 8'd0;
    assign diag_commit_gray=DIAG_EN ? commit_gray : 8'd0;
    assign diag_drop_gray=DIAG_EN ? drop_gray : 8'd0;
    rd_data_fifo rd_data_fifo(
      .Data(app_rd_data),.Reset(!rst_n || rs==R_RESET),.WrClk(ui_clk),.RdClk(rd_clk),
      .WrEn(app_rd_data_valid),.RdEn(rfifo_rden),.Wnum(rfifo_wcount),.Rnum(),
      .Almost_Empty(),.Almost_Full(),.Q(rfifo_dout),.Empty(rdfifo_empty),.Full());
    wr_data_fifo wr_data_fifo(
      .Data(wfifo_din),.Reset(fifo_reset),.WrClk(wr_clk),.RdClk(ui_clk),
      .WrEn(wfifo_wren && capture_on && !wrfifo_full),.RdEn(write_fire),.Wnum(),
      .Rnum(wfifo_rcount),.Almost_Empty(),.Almost_Full(),.Q(wfifo_q),.Empty(wfifo_empty),.Full(wrfifo_full));
endmodule

