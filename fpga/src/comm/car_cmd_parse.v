/*
 * car_cmd_parse.v
 * A5/5A协议帧解析：检查长度和CRC并发布完整命令。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module car_cmd_parse (
    input             Clk,
    input             Rst_n,

    input      [7:0]  RxData,
    input             RxValid,       

    output reg        CmdValid,      
    output reg [7:0]  CmdCode,
    output reg [7:0]  CmdSeq,
    output reg [7:0]  CmdLen,
    output reg [15:0] CmdArg0,
    output reg [15:0] CmdArg1,
    output reg [15:0] CmdArg2,
    output reg [15:0] CmdArg3,
    output reg [15:0] CmdArg4,
    output reg [15:0] CmdArg5,

    output reg        LinkAlive,     
    output reg [15:0] OkCnt,         
    output reg [15:0] ErrCnt         
);

    localparam S_HEAD_H = 3'd0,    
               S_HEAD_L = 3'd1,    
               S_LEN    = 3'd2,
               S_SEQ    = 3'd3,
               S_CMD    = 3'd4,
               S_DATA   = 3'd5,
               S_CRC    = 3'd6;

    localparam integer BUF_MAX = 12;    
    localparam integer GAP_CNT = (`CLK_CTRL_HZ / 1000) * `FRAME_TIMEOUT_MS;   

    reg [2:0]  state;
    reg [7:0]  len_r;
    reg [7:0]  seq_r;
    reg [7:0]  cmd_r;
    reg [7:0]  dat_cnt;               
    reg [7:0]  dat_buf [0:BUF_MAX-1]; 
    reg [7:0]  crc_r;                 
    reg [31:0] gap_cnt;               

    function [7:0] crc8_byte;
        input [7:0] crc_in;
        input [7:0] byte_in;
        integer i;
        reg [7:0] c;
        begin
            c = crc_in ^ byte_in;
            for (i = 0; i < 8; i = i + 1) begin
                if (c[7]) c = {c[6:0], 1'b0} ^ 8'h07;
                else      c = {c[6:0], 1'b0};
            end
            crc8_byte = c;
        end
    endfunction

    wire gap_timeout = (gap_cnt >= GAP_CNT - 1);

    integer k;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        state     <= S_HEAD_H;
        len_r     <= 8'd0;
        seq_r     <= 8'd0;
        cmd_r     <= 8'd0;
        dat_cnt   <= 8'd0;
        crc_r     <= 8'h00;
        gap_cnt   <= 32'd0;
        CmdValid  <= 1'b0;
        CmdCode   <= 8'h00;
        CmdSeq    <= 8'd0;
        CmdLen    <= 8'd0;
        CmdArg0   <= 16'd0;
        CmdArg1   <= 16'd0;
        CmdArg2   <= 16'd0;
        CmdArg3   <= 16'd0;
        CmdArg4   <= 16'd0;
        CmdArg5   <= 16'd0;
        LinkAlive <= 1'b0;
        OkCnt     <= 16'd0;
        ErrCnt    <= 16'd0;
        for (k = 0; k < BUF_MAX; k = k + 1) dat_buf[k] <= 8'h00;
    end
    else begin
        CmdValid <= 1'b0;                        

        if(RxValid) gap_cnt <= 32'd0;
        else if(!gap_timeout) gap_cnt <= gap_cnt + 1'b1;

        if(gap_timeout && (state != S_HEAD_H)) begin
            state   <= S_HEAD_H;
            dat_cnt <= 8'd0;
            ErrCnt  <= ErrCnt + 1'b1;
        end
        else if(RxValid) begin
            case(state)

            S_HEAD_H: begin
                if(RxData == `FRAME_HEAD_H) state <= S_HEAD_L;
            end

            S_HEAD_L: begin
                if(RxData == `FRAME_HEAD_L) begin
                    state <= S_LEN;
                end
                else if(RxData == `FRAME_HEAD_H) begin
                    state <= S_HEAD_L;           
                end
                else state <= S_HEAD_H;
            end

            S_LEN: begin
                if((RxData >= `FRAME_LEN_MIN) && (RxData <= `FRAME_LEN_MAX)) begin
                    len_r  <= RxData;
                    crc_r  <= crc8_byte(`CRC8_INIT, RxData);    
                    state  <= S_SEQ;
                end
                else begin
                    ErrCnt <= ErrCnt + 1'b1;
                    state  <= S_HEAD_H;
                end
            end

            S_SEQ: begin
                seq_r <= RxData;
                crc_r <= crc8_byte(crc_r, RxData);
                state <= S_CMD;
            end

            S_CMD: begin
                cmd_r   <= RxData;
                crc_r   <= crc8_byte(crc_r, RxData);
                dat_cnt <= 8'd0;

                state   <= (len_r == 8'd1) ? S_CRC : S_DATA;
            end

            S_DATA: begin
                if(dat_cnt < BUF_MAX) dat_buf[dat_cnt] <= RxData;
                crc_r   <= crc8_byte(crc_r, RxData);
                if(dat_cnt >= len_r - 8'd2) state <= S_CRC;    
                else                        dat_cnt <= dat_cnt + 1'b1;
            end

            S_CRC: begin
                if(RxData == crc_r) begin

                    CmdCode <= cmd_r;
                    CmdSeq  <= seq_r;
                    CmdLen  <= len_r;
                    CmdArg0 <= {dat_buf[1], dat_buf[0]};
                    CmdArg1 <= {dat_buf[3], dat_buf[2]};
                    CmdArg2 <= {dat_buf[5], dat_buf[4]};
                    CmdArg3 <= {dat_buf[7], dat_buf[6]};
                    CmdArg4 <= {dat_buf[9], dat_buf[8]};
                    CmdArg5 <= {dat_buf[11], dat_buf[10]};
                    CmdValid  <= 1'b1;
                    LinkAlive <= 1'b1;
                    OkCnt     <= OkCnt + 1'b1;
                end
                else begin
                    ErrCnt <= ErrCnt + 1'b1;
                end
                state <= S_HEAD_H;
            end

            default: state <= S_HEAD_H;
            endcase
        end

        if(`LEGACY_1BYTE_EN && RxValid && (state == S_HEAD_H)) begin
            case(RxData)
            8'h1A: begin CmdValid <= 1'b1; CmdCode <= `CMD_MOTION; CmdArg0 <= {8'd50, `DIR_FWD   }; end
            8'h2A: begin CmdValid <= 1'b1; CmdCode <= `CMD_MOTION; CmdArg0 <= {8'd0,  `DIR_STOP  }; end
            8'h3A: begin CmdValid <= 1'b1; CmdCode <= `CMD_MOTION; CmdArg0 <= {8'd50, `DIR_SPIN_L}; end
            8'h4A: begin CmdValid <= 1'b1; CmdCode <= `CMD_MOTION; CmdArg0 <= {8'd50, `DIR_SPIN_R}; end
            default: ;
            endcase
        end
    end

endmodule
