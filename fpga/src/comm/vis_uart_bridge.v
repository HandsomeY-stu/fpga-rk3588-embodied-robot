/*
 * vis_uart_bridge.v
 * 远程视觉/控制命令分发，处理模式请求、重复命令和回包。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module vis_uart_bridge #(
    parameter CLK_FREQ       = 50_000_000,
    parameter VIS_TIMEOUT_MS = 500,          

    parameter LINK_TIMEOUT_MS = 500,         
    parameter ALLOW_ROTATE    = 0,           

    parameter REMOTE_DEDUP_MS = 300
)(
    input             Clk,           
    input             Rst_n,

    input      [7:0]  RxData,
    input             RxValid,       

    output reg [7:0]  TxData,
    output reg        TxValid,       
    input             TxBusy,

    output reg        VisValid,      
    output reg [7:0]  VisCls,
    output reg [15:0] VisCx,
    output reg [15:0] VisCy,
    output reg [15:0] VisW,
    output reg [15:0] VisH,
    output reg [7:0]  VisConf,
    output reg [7:0]  VisSeq,

    output reg        OsdValid,      
    output reg [1:0]  OsdMode,       

    output reg [15:0] PingCnt,       
    output reg [15:0] PongCnt,       
    output reg [15:0] VisCnt,        
    output wire       LinkAlive,     
    output wire [15:0] ErrCnt,       

    output reg        MoValid,       
    output reg [7:0]  MoCode,        
    output reg [15:0] MoArg0,        
    output reg [15:0] MoArg1,
    output reg [15:0] MoArg2,
    output reg [7:0]  MoSeq,         
    output reg        MoRejected,    
    output reg [7:0]  MoLastCode,    

    output reg        MoLinkLive,
    output reg [15:0] MoGapMs,       

    output reg        StartT3,       
    output reg        StartT4        
);

    wire        p_valid;
    wire [7:0]  p_code, p_seq, p_len;
    wire [15:0] p_arg0, p_arg1, p_arg2, p_arg3, p_arg4, p_arg5;
    wire        p_alive;
    wire [15:0] p_ok, p_err;

    car_cmd_parse u_vis_parse (
        .Clk      (Clk),
        .Rst_n    (Rst_n),
        .RxData   (RxData),
        .RxValid  (RxValid),
        .CmdValid (p_valid),
        .CmdCode  (p_code),
        .CmdSeq   (p_seq),
        .CmdLen   (p_len),
        .CmdArg0  (p_arg0),
        .CmdArg1  (p_arg1),
        .CmdArg2  (p_arg2),
        .CmdArg3  (p_arg3),
        .CmdArg4  (p_arg4),
        .CmdArg5  (p_arg5),
        .LinkAlive(p_alive),
        .OkCnt    (p_ok),
        .ErrCnt   (p_err)
    );

    assign ErrCnt    = p_err;
    assign LinkAlive = p_alive;

    localparam integer VIS_TIMEOUT_CNT = (CLK_FREQ/1000) * VIS_TIMEOUT_MS;
    reg [31:0] vis_gap;
    wire vis_timeout = (vis_gap >= VIS_TIMEOUT_CNT - 1);

    localparam integer LINK_TIMEOUT_CNT = (CLK_FREQ/1000) * LINK_TIMEOUT_MS;
    reg [31:0] link_gap;
    reg [31:0] ms_cnt;
    localparam integer MS_DIV = (CLK_FREQ/1000);
    wire ms_tick = (ms_cnt >= MS_DIV - 1);

    localparam integer REMOTE_DEDUP_CNT = (CLK_FREQ/1000) * REMOTE_DEDUP_MS;
    reg [31:0] rmt_gap;
    wire       rmt_gap_ok = (rmt_gap >= REMOTE_DEDUP_CNT - 1);

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        OsdValid <= 1'b0;
        OsdMode  <= 2'd1;
    end
    else begin
        OsdValid <= p_valid && (p_code == `CMD_OSD_MODE);
        if(p_valid && (p_code == `CMD_OSD_MODE)) OsdMode <= p_arg0[1:0];
    end

    wire p_mode_is_start = (p_code == `CMD_MODE) &&
                           ((p_arg0[7:0] == `MODE_AVOID) ||
                            (p_arg0[7:0] == `MODE_SEG1 ) ||
                            (p_arg0[7:0] == `MODE_NAV  ));

    reg [7:0]  pb_seq, pb_echo;               
    reg [7:0]  vb_seq, vb_cls, vb_conf;       
    reg [15:0] vb_cx, vb_cy, vb_w, vb_h;

    reg [7:0]  ts_seq, ts_echo, ts_cls, ts_conf;
    reg [15:0] ts_cx, ts_cy, ts_w, ts_h;

    localparam integer TX_BUF = 16;
    reg [7:0]  tx_buf [0:TX_BUF-1];
    reg [4:0]  tx_len;           
    reg [4:0]  tx_idx;
    reg [1:0]  tx_st;            
    reg        tx_frame_vis;     
    reg        tx_last;          

    reg        req_pend;         
    reg        req_is_vis;       
    reg [7:0]  req_echo;

    function [7:0] crc8_byte;
        input [7:0] c_in; input [7:0] b_in;
        integer i; reg [7:0] c;
        begin
            c = c_in ^ b_in;
            for (i = 0; i < 8; i = i + 1)
                c = c[7] ? ({c[6:0],1'b0} ^ 8'h07) : {c[6:0],1'b0};
            crc8_byte = c;
        end
    endfunction

    function [7:0] pong_byte;
        input [4:0] i;
        reg [7:0] c;
        begin
            case(i)
            5'd0: pong_byte = `FRAME_HEAD_H;
            5'd1: pong_byte = `FRAME_HEAD_L;
            5'd2: pong_byte = 8'd2;
            5'd3: pong_byte = ts_seq;
            5'd4: pong_byte = `TLM_PONG;
            5'd5: pong_byte = ts_echo;
            5'd6: begin
                c = crc8_byte(`CRC8_INIT, 8'd2);
                c = crc8_byte(c, ts_seq);
                c = crc8_byte(c, `TLM_PONG);
                c = crc8_byte(c, ts_echo);
                pong_byte = c;
            end
            default: pong_byte = 8'h00;
            endcase
        end
    endfunction

    function [7:0] vis_byte;
        input [4:0] i;
        reg [7:0] c;
        begin
            case(i)
            5'd0 : vis_byte = `FRAME_HEAD_H;
            5'd1 : vis_byte = `FRAME_HEAD_L;
            5'd2 : vis_byte = 8'd11;
            5'd3 : vis_byte = ts_seq;
            5'd4 : vis_byte = `TLM_VISION;
            5'd5 : vis_byte = ts_cls;
            5'd6 : vis_byte = ts_cx[7:0];
            5'd7 : vis_byte = ts_cx[15:8];
            5'd8 : vis_byte = ts_cy[7:0];
            5'd9 : vis_byte = ts_cy[15:8];
            5'd10: vis_byte = ts_w[7:0];
            5'd11: vis_byte = ts_w[15:8];
            5'd12: vis_byte = ts_h[7:0];
            5'd13: vis_byte = ts_h[15:8];
            5'd14: vis_byte = ts_conf;
            5'd15: begin
                c = crc8_byte(`CRC8_INIT, 8'd11);
                c = crc8_byte(c, ts_seq);
                c = crc8_byte(c, `TLM_VISION);
                c = crc8_byte(c, ts_cls);
                c = crc8_byte(c, ts_cx[7:0]);  c = crc8_byte(c, ts_cx[15:8]);
                c = crc8_byte(c, ts_cy[7:0]);  c = crc8_byte(c, ts_cy[15:8]);
                c = crc8_byte(c, ts_w[7:0]);   c = crc8_byte(c, ts_w[15:8]);
                c = crc8_byte(c, ts_h[7:0]);   c = crc8_byte(c, ts_h[15:8]);
                c = crc8_byte(c, ts_conf);
                vis_byte = c;
            end
            default: vis_byte = 8'h00;
            endcase
        end
    endfunction

    wire [7:0] cur_byte = tx_frame_vis ? vis_byte(tx_idx) : pong_byte(tx_idx);

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        TxData  <= 8'd0;  TxValid <= 1'b0;
        tx_idx  <= 5'd0;  tx_st   <= 2'd0;  tx_frame_vis <= 1'b0;
        tx_len  <= 5'd0;  tx_last <= 1'b0;
        req_pend<= 1'b0;  req_echo <= 8'd0; req_is_vis <= 1'b0;
        pb_seq  <= 8'd0;  pb_echo <= 8'd0;
        vb_seq  <= 8'd0;  vb_cls  <= 8'd0;  vb_conf <= 8'd0;
        vb_cx   <= 16'd0; vb_cy <= 16'd0; vb_w <= 16'd0; vb_h <= 16'd0;
        ts_seq  <= 8'd0;  ts_echo <= 8'd0;  ts_cls <= 8'd0; ts_conf <= 8'd0;
        ts_cx   <= 16'd0; ts_cy <= 16'd0; ts_w <= 16'd0; ts_h <= 16'd0;
        VisValid<= 1'b0;
        VisCls  <= 8'd0;  VisCx <= 16'd0; VisCy <= 16'd0;
        VisW    <= 16'd0; VisH  <= 16'd0; VisConf <= 8'd0; VisSeq <= 8'd0;
        PingCnt <= 16'd0; PongCnt <= 16'd0; VisCnt <= 16'd0;
        vis_gap <= 32'd0;

        MoValid  <= 1'b0; MoCode <= 8'd0;
        MoArg0   <= 16'd0; MoArg1 <= 16'd0; MoArg2 <= 16'd0;
        MoSeq    <= 8'd0; MoRejected <= 1'b0; MoLastCode <= 8'd0;
        link_gap <= 32'd0; ms_cnt <= 32'd0;
        MoLinkLive <= 1'b0; MoGapMs <= 16'd0;

        StartT3  <= 1'b0; StartT4 <= 1'b0;
        rmt_gap  <= REMOTE_DEDUP_CNT[31:0];
    end
    else begin
        TxValid <= 1'b0;

        if(p_valid && (p_code == `CMD_VISION)) vis_gap <= 32'd0;
        else if(!vis_timeout)                   vis_gap <= vis_gap + 1'b1;

        if(vis_timeout) VisValid <= 1'b0;

        if(p_valid) begin
            if(p_code == `CMD_PING) begin
                PingCnt <= PingCnt + 1'b1;
                pb_seq  <= p_seq;
                pb_echo <= p_seq;
                req_pend<= 1'b1;
                req_is_vis <= 1'b0;
            end
            else if(p_code == `CMD_VISION) begin
                VisCls  <= p_arg0[7:0];
                VisCx   <= {p_arg1[7:0],  p_arg0[15:8]};
                VisCy   <= {p_arg2[7:0],  p_arg1[15:8]};
                VisW    <= {p_arg3[7:0],  p_arg2[15:8]};
                VisH    <= {p_arg4[7:0],  p_arg3[15:8]};
                VisConf <= p_arg4[15:8];
                VisSeq  <= p_seq;
                VisValid<= 1'b1;
                VisCnt  <= VisCnt + 1'b1;

                vb_seq  <= p_seq;
                vb_cls  <= p_arg0[7:0];
                vb_cx   <= {p_arg1[7:0],  p_arg0[15:8]};
                vb_cy   <= {p_arg2[7:0],  p_arg1[15:8]};
                vb_w    <= {p_arg3[7:0],  p_arg2[15:8]};
                vb_h    <= {p_arg4[7:0],  p_arg3[15:8]};
                vb_conf <= p_arg4[15:8];
                req_pend<= 1'b1;
                req_is_vis <= 1'b1;    
            end
        end

        StartT3 <= 1'b0;
        StartT4 <= 1'b0;

        if(rmt_gap < REMOTE_DEDUP_CNT) rmt_gap <= rmt_gap + 1'b1;
        if(p_valid && p_mode_is_start && rmt_gap_ok) begin
            StartT3 <= (p_arg0[7:0] == `MODE_AVOID);
            StartT4 <= (p_arg0[7:0] != `MODE_AVOID);
            rmt_gap <= 32'd0;
        end

        MoValid    <= 1'b0;
        MoRejected <= 1'b0;
        if(p_valid) begin

            if((p_code == `CMD_MOTION) || (p_code == `CMD_SPEED) || (p_code == `CMD_STOP) ||
               (p_code == `CMD_BEEP) ||
               ((p_code == `CMD_MODE) && (p_arg0[7:0] == `MODE_MANUAL)) ||
               ((p_code == `CMD_ROTATE) && (ALLOW_ROTATE != 0))) begin
                MoValid <= 1'b1;
                MoCode  <= p_code;
                MoArg0  <= p_arg0;
                MoArg1  <= p_arg1;
                MoArg2  <= p_arg2;
                MoSeq   <= p_seq;
            end
            else if(((p_code == `CMD_MODE) && !p_mode_is_start) ||
                    (p_code == `CMD_MOTION) || (p_code == `CMD_SPEED) ||
                    (p_code == `CMD_ROTATE) || (p_code == `CMD_WAYPOINT) ||
                    (p_code == `CMD_NAV_START) || (p_code == `CMD_NAV_ABORT)) begin

                MoRejected <= 1'b1;
                MoLastCode <= p_code;
            end
        end

        if(p_valid) link_gap <= 32'd0;
        else if(!(link_gap >= LINK_TIMEOUT_CNT - 1)) link_gap <= link_gap + 1'b1;

        MoLinkLive <= (link_gap < LINK_TIMEOUT_CNT - 1);

        if(ms_tick) ms_cnt <= 32'd0;
        else        ms_cnt <= ms_cnt + 1'b1;

        if(p_valid)                        MoGapMs <= 16'd0;
        else if(ms_tick && (MoGapMs != 16'hFFFF)) MoGapMs <= MoGapMs + 16'd1;

        case(tx_st)

        2'd0: begin  
            if(req_pend) begin
                tx_frame_vis <= req_is_vis;
                ts_seq  <= req_is_vis ? vb_seq  : pb_seq;
                ts_echo <= pb_echo;
                ts_cls  <= vb_cls;
                ts_conf <= vb_conf;
                ts_cx   <= vb_cx;  ts_cy <= vb_cy;
                ts_w    <= vb_w;   ts_h  <= vb_h;
                tx_idx  <= 5'd0;
                tx_last <= 1'b0;
                tx_st   <= 2'd1;
                req_pend<= 1'b0;
            end
        end

        2'd1: begin  
            TxData <= cur_byte;
            if(TxBusy) begin

                TxValid <= 1'b0;
                if(tx_idx == (tx_frame_vis ? 5'd15 : 5'd6)) tx_last <= 1'b1;
                else                                        tx_idx  <= tx_idx + 1'b1;
                tx_st   <= 2'd2;
            end
            else begin
                TxValid <= 1'b1;
            end
        end

        default: begin  
            if(!TxBusy) begin
                if(tx_last) begin
                    tx_st    <= 2'd0;
                    tx_last  <= 1'b0;
                    PongCnt  <= PongCnt + 1'b1;
                end
                else tx_st <= 2'd1;
            end
        end
        endcase
    end

endmodule
