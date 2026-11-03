/*
 * vis_osd_text.v
 * 视觉检测结果转字符文本，为显示叠加提供目标信息。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
module vis_osd_text #(
    parameter integer COLS = 13         
)(
    input             Clk,              
    input             Rst_n,

    input             VisValid,         
    input      [7:0]  VisCls,           
    input      [15:0] VisCx,            
    input      [15:0] VisCy,            
    input      [7:0]  VisConf,          
    input      [15:0] VisCnt,           
    input      [3:0]  FsmState,         

    output reg [COLS*8*3-1:0] TxtA,     
    output reg [COLS*8*3-1:0] TxtB,     
    output reg        BankSel            
);

    function [7:0] fsm_ch;
        input [3:0] s;
        begin
            case (s)
            4'd0:    fsm_ch = 8'h49;     
            4'd1:    fsm_ch = 8'h4C;     
            4'd2:    fsm_ch = 8'h52;     
            4'd3:    fsm_ch = 8'h46;     
            4'd4:    fsm_ch = 8'h4E;     
            default: fsm_ch = 8'h3F;     
            endcase
        end
    endfunction

    function [7:0] hexch;
        input [3:0] n;
        begin
            hexch = (n < 4'd10) ? (8'h30 + {4'd0, n})
                                : (8'h41 + {4'd0, n} - 8'd10);
        end
    endfunction

    localparam LINE_CH = 13;

    function [COLS*8-1:0] place_line;
        input [103:0] ln;                
        integer i;
        begin
            place_line = {COLS{8'h20}};
            for (i = 0; i < LINE_CH; i = i + 1)
                place_line[i*8 +: 8] = ln[i*8 +: 8];
        end
    endfunction

    wire [103:0] ln0 = {
        8'h20, 8'h20,                                     
        hexch(VisCx[3:0]),  hexch(VisCx[7:4]),            
        hexch(VisCx[11:8]), hexch(VisCx[15:12]),          
        8'h3A, 8'h58,                                     
        8'h20,                                            
        hexch(VisCls[3:0]),  hexch(VisCls[7:4]),          
        8'h3A, 8'h43                                      
    };

    wire [103:0] ln1 = {
        8'h20, 8'h20,                                     
        hexch(VisConf[3:0]),  hexch(VisConf[7:4]),        
        8'h3A, 8'h51,                                     
        8'h20,                                            
        hexch(VisCy[3:0]),  hexch(VisCy[7:4]),            
        hexch(VisCy[11:8]), hexch(VisCy[15:12]),          
        8'h3A, 8'h59                                      
    };

    wire [103:0] ln2 = {
        8'h20, 8'h20, 8'h20,                              
        hexch(VisCnt[3:0]),  hexch(VisCnt[7:4]),          
        hexch(VisCnt[11:8]), hexch(VisCnt[15:12]),        
        8'h3A, 8'h4E,                                     
        8'h20,                                            
        fsm_ch(FsmState),                                 
        8'h3A, 8'h53                                      
    };

    wire [COLS*8*3-1:0] txt_next = {
        place_line(ln2), place_line(ln1), place_line(ln0)
    };

    localparam [103:0] INIT_L0 = {
        8'h2D, 8'h2D, 8'h2D, 8'h2D, 8'h3A, 8'h58, 8'h20,    
        8'h2D, 8'h2D, 8'h3A, 8'h43                             
    };

    localparam [103:0] INIT_L1 = {
        8'h2D, 8'h2D, 8'h3A, 8'h51, 8'h20,                  
        8'h2D, 8'h2D, 8'h2D, 8'h2D, 8'h3A, 8'h59             
    };

    localparam [103:0] INIT_L2 = {
        8'h2D, 8'h2D, 8'h2D, 8'h2D, 8'h3A, 8'h4E, 8'h20,    
        8'h2D, 8'h3A, 8'h53                                  
    };

    wire [COLS*8*3-1:0] TXT_INIT = {
        place_line(INIT_L2), place_line(INIT_L1), place_line(INIT_L0)
    };

    reg wr_bank;
    reg prev_vis_valid;
    reg [1:0] trig_dly;
    reg cnt_lsb_d;                                
    wire vis_pulse = VisValid && !prev_vis_valid;

    wire cnt_evt   = VisCnt[0] ^ cnt_lsb_d;
    wire frame_evt = vis_pulse | cnt_evt;
    wire vis_go    = trig_dly[1];    

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        TxtA     <= TXT_INIT;
        TxtB     <= TXT_INIT;
        wr_bank  <= 1'b0;
        BankSel  <= 1'b0;
        prev_vis_valid <= 1'b0;
        trig_dly <= 2'b00;
        cnt_lsb_d <= 1'b0;
    end
    else begin
        prev_vis_valid <= VisValid;
        cnt_lsb_d <= VisCnt[0];
        trig_dly <= {trig_dly[0], frame_evt};    
        if(vis_go) begin
            if(!wr_bank) TxtA <= txt_next;
            else         TxtB <= txt_next;
            BankSel <= wr_bank;
            wr_bank <= ~wr_bank;
        end
    end

endmodule
