/*
 * osd_overlay.v
 * 像素流OSD字符叠加，生成姿态角等文字的显示区域。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
module osd_overlay #(

    parameter integer DUAL_TEXT = 0,           
    parameter integer ROWS     = 4,             
    parameter integer COLS     = 13,            
    parameter integer SCALE_LG = 2,             
    parameter [11:0]  X0       = 12'd32,        
    parameter [11:0]  Y0       = 12'd32,        
    parameter [23:0]  COL_FG   = 24'hFFFFFF,    
    parameter [23:0]  COL_BG   = 24'h000000,    
    parameter [23:0]  COL_BRD  = 24'h20A020     
)(
    input             PixClk,                   
    input             Rst_n,
    input      [11:0] Hcount,                   
    input      [11:0] Vcount,                   
    input      [23:0] Din,                      
    input      [ROWS*COLS*8-1:0] Txt,           
    input      [ROWS*COLS*8-1:0] TxtAlt,
    input             TxtBank,
    output     [23:0] Dout                      
);

    localparam CELL_W  = 8 << SCALE_LG;         
    localparam CELL_H  = 8 << SCALE_LG;         
    localparam TEXT_W  = COLS * CELL_W;         
    localparam TEXT_H  = ROWS * CELL_H;         
    localparam LOG2_CW = SCALE_LG + 3;          
    localparam PAD     = 8;                     
    localparam BORDER  = 2;                     

    localparam [11:0] PAD12 = PAD;              

    localparam BOXW = TEXT_W + 2 * PAD;         
    localparam BOXH = TEXT_H + 2 * PAD;         

    localparam integer NCH = ROWS * COLS;       

    function [34:0] osd_glyph;
        input [7:0] ch;
        begin
            case (ch)
            8'h20: osd_glyph = 35'b00000_00000_00000_00000_00000_00000_00000;   
            8'h2B: osd_glyph = 35'b00000_00100_00100_11111_00100_00100_00000;   
            8'h2D: osd_glyph = 35'b00000_00000_00000_11111_00000_00000_00000;   
            8'h2E: osd_glyph = 35'b00000_00000_00000_00000_00000_01100_01100;   
            8'h30: osd_glyph = 35'b01110_10001_10011_10101_11001_10001_01110;   
            8'h31: osd_glyph = 35'b00100_01100_00100_00100_00100_00100_01110;   
            8'h32: osd_glyph = 35'b01110_10001_00001_00010_00100_01000_11111;   
            8'h33: osd_glyph = 35'b01110_10001_00001_00110_00001_10001_01110;   
            8'h34: osd_glyph = 35'b00010_00110_01010_10010_11111_00010_00010;   
            8'h35: osd_glyph = 35'b11111_10000_11110_00001_00001_10001_01110;   
            8'h36: osd_glyph = 35'b00110_01000_10000_11110_10001_10001_01110;   
            8'h37: osd_glyph = 35'b11111_00001_00010_00100_01000_01000_01000;   
            8'h38: osd_glyph = 35'b01110_10001_10001_01110_10001_10001_01110;   
            8'h39: osd_glyph = 35'b01110_10001_10001_01111_00001_00010_01100;   
            8'h3A: osd_glyph = 35'b00000_01100_01100_00000_01100_01100_00000;   
            8'h41: osd_glyph = 35'b00100_01010_10001_10001_11111_10001_10001;   
            8'h42: osd_glyph = 35'b11110_10001_10001_11110_10001_10001_11110;   
            8'h43: osd_glyph = 35'b01110_10001_10000_10000_10000_10001_01110;   
            8'h44: osd_glyph = 35'b11100_10010_10001_10001_10001_10010_11100;   
            8'h45: osd_glyph = 35'b11111_10000_10000_11110_10000_10000_11111;   
            8'h46: osd_glyph = 35'b11111_10000_10000_11110_10000_10000_10000;   
            8'h47: osd_glyph = 35'b01110_10001_10000_10111_10001_10001_01110;   
            8'h48: osd_glyph = 35'b10001_10001_10001_11111_10001_10001_10001;   
            8'h49: osd_glyph = 35'b01110_00100_00100_00100_00100_00100_01110;   
            8'h4A: osd_glyph = 35'b00111_00010_00010_00010_00010_10010_01100;   
            8'h4B: osd_glyph = 35'b10001_10010_10100_11000_10100_10010_10001;   
            8'h4C: osd_glyph = 35'b10000_10000_10000_10000_10000_10000_11111;   
            8'h4D: osd_glyph = 35'b10001_11011_10101_10001_10001_10001_10001;   
            8'h4E: osd_glyph = 35'b10001_11001_10101_10011_10001_10001_10001;   
            8'h4F: osd_glyph = 35'b01110_10001_10001_10001_10001_10001_01110;   
            8'h50: osd_glyph = 35'b11110_10001_10001_11110_10000_10000_10000;   
            8'h51: osd_glyph = 35'b01110_10001_10001_10001_10101_10010_01101;   
            8'h52: osd_glyph = 35'b11110_10001_10001_11110_10100_10010_10001;   
            8'h53: osd_glyph = 35'b01111_10000_10000_01110_00001_00001_11110;   
            8'h54: osd_glyph = 35'b11111_00100_00100_00100_00100_00100_00100;   
            8'h55: osd_glyph = 35'b10001_10001_10001_10001_10001_10001_01110;   
            8'h56: osd_glyph = 35'b10001_10001_10001_10001_10001_01010_00100;   
            8'h57: osd_glyph = 35'b10001_10001_10001_10101_10101_11011_10001;   
            8'h58: osd_glyph = 35'b10001_10001_01010_00100_01010_10001_10001;   
            8'h59: osd_glyph = 35'b10001_10001_01010_00100_00100_00100_00100;   
            8'h5A: osd_glyph = 35'b11111_00001_00010_00100_01000_10000_11111;   
            default: osd_glyph = 35'd0;
            endcase
        end
    endfunction

    function [4:0] osd_row;                     
        input [34:0] g;
        input [2:0]  r;
        begin
            case (r)
            3'd0: osd_row = g[34:30];
            3'd1: osd_row = g[29:25];
            3'd2: osd_row = g[24:20];
            3'd3: osd_row = g[19:15];
            3'd4: osd_row = g[14:10];
            3'd5: osd_row = g[9:5];
            3'd6: osd_row = g[4:0];
            default: osd_row = 5'b0;
            endcase
        end
    endfunction

    wire [11:0] dx = Hcount - X0;               
    wire [11:0] dy = Vcount - Y0;               
    wire [11:0] tx = dx - PAD12;                
    wire [11:0] ty = dy - PAD12;

    reg [11:0] dx1, dx2, dx3, dx4;
    reg [11:0] dy1, dy2, dy3, dy4;
    reg [11:0] tx1, tx2, tx3, tx4;
    reg [11:0] ty1, ty2, ty3, ty4;

    localparam integer ROW_BITS=(ROWS>1) ? $clog2(ROWS) : 1;
    localparam integer COL_BITS=(COLS>1) ? $clog2(COLS) : 1;
    wire [ROW_BITS-1:0] linc = ty1[LOG2_CW +: ROW_BITS];
    wire [COL_BITS-1:0] colc = tx1[LOG2_CW +: COL_BITS];

    wire text_index_valid = (tx1 < TEXT_W) && (ty1 < TEXT_H);
    wire [7:0] ridx_calc = linc * COLS + colc;
    wire [7:0] ridx_w = text_index_valid ? ridx_calc : 8'd0;

    reg [7:0]  ridx;                            
    reg [7:0]  rch;
    reg [7:0] rch_alt;
    reg rch_bank;
    reg [34:0] rgl;

    always @(posedge PixClk or negedge Rst_n)
    if(!Rst_n) begin
        dx1 <= 12'd0; dx2 <= 12'd0; dx3 <= 12'd0;
        dy1 <= 12'd0; dy2 <= 12'd0; dy3 <= 12'd0;
        tx1 <= 12'd0; tx2 <= 12'd0; tx3 <= 12'd0;
        ty1 <= 12'd0; ty2 <= 12'd0; ty3 <= 12'd0;
        dx4 <= 0;dy4 <= 0;tx4 <= 0;ty4 <= 0;
        ridx <= 8'd0; rch <= 8'h20; rch_alt <= 8'h20; rch_bank <= 0; rgl <= 35'd0;
    end
    else begin

        dx1  <= dx;
        dy1  <= dy;
        tx1  <= tx;
        ty1  <= ty;
        ridx <= ridx_w;                         

        dx2  <= dx1;
        dy2  <= dy1;
        tx2  <= tx1;
        ty2  <= ty1;
        rch  <= Txt[8*ridx +: 8];
        if(DUAL_TEXT) begin
            rch_alt <= TxtAlt[8*ridx +: 8];
            rch_bank <= TxtBank;
        end

        dx3  <= dx2;
        dy3  <= dy2;
        tx3  <= tx2;
        ty3  <= ty2;
        rgl  <= osd_glyph(DUAL_TEXT && rch_bank ? rch_alt : rch);
        dx4 <= dx3;dy4 <= dy3;tx4 <= tx3;ty4 <= ty3;
    end

    wire in_box   = (dx4 < BOXW) && (dy4 < BOXH);
    wire in_inner = (dx4 >= BORDER) && (dx4 < BOXW - BORDER) &&
                    (dy4 >= BORDER) && (dy4 < BOXH - BORDER);
    wire on_border = in_box && !in_inner;

    wire in_text  = (tx4 < TEXT_W) && (ty4 < TEXT_H);

    wire [3:0] fx  = (tx4 >> SCALE_LG) & 4'd7;    
    wire [3:0] fy  = (ty4 >> SCALE_LG) & 4'd7;    
    wire [4:0] rb  = osd_row(rgl, fy[2:0]);       

    function [0:0] osd_px_on;
        input [4:0] row;
        input [3:0] fx;
        begin
            case (fx)
            4'd1:    osd_px_on = row[4];
            4'd2:    osd_px_on = row[3];
            4'd3:    osd_px_on = row[2];
            4'd4:    osd_px_on = row[1];
            4'd5:    osd_px_on = row[0];
            default: osd_px_on = 1'b0;            
            endcase
        end
    endfunction

    wire px_on = osd_px_on(rb, fx);

    wire [23:0] osd_px = on_border        ? COL_BRD :
                         (in_text && px_on) ? COL_FG  : COL_BG;

    assign Dout = in_box ? osd_px : Din;

endmodule
