/*
 * angle_text_fmt.v
 * 将姿态角转换为定长字符文本，使用双缓冲发布完整文本。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
module angle_text_fmt #(

    parameter OSD_DBG_EN = 0,

    parameter integer COLS = 13
)(
    input             Clk,           
    input             Rst_n,
    input             AngValid,      
    input             LinkOk,        
    input      [15:0] RollRaw,       
    input      [15:0] PitchRaw,
    input      [15:0] YawRaw,
    input      [15:0] DbgOk,         
    input      [15:0] DbgErr,        
    input      [63:0] DbgBytes,      
    output reg [COLS*8*4-1:0] TxtA,  
    output reg [COLS*8*4-1:0] TxtB,  
    output reg        BankSel        
);

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

    localparam [COLS*8-1:0] PL_BLANK = {COLS{8'h20}};    

    localparam [103:0] INIT_OFF_0 = {
        8'h2D, 8'h2D, 8'h2E, 8'h2D, 8'h2D, 8'h20, 8'h20, 8'h3A,
        8'h20, 8'h4C, 8'h4C, 8'h4F, 8'h52
    };

    localparam [103:0] INIT_OFF_1 = {
        8'h2D, 8'h2D, 8'h2E, 8'h2D, 8'h2D, 8'h20, 8'h20, 8'h3A,
        8'h48, 8'h43, 8'h54, 8'h49, 8'h50
    };

    localparam [103:0] INIT_OFF_2 = {
        8'h2D, 8'h2D, 8'h2E, 8'h2D, 8'h2D, 8'h20, 8'h20, 8'h3A,
        8'h20, 8'h20, 8'h57, 8'h41, 8'h59
    };

    localparam [103:0] INIT_DBG_0 = {
        8'h20, 8'h2D, 8'h2D, 8'h2D, 8'h2D, 8'h2D, 8'h2D, 8'h2D,
        8'h2D, 8'h3A, 8'h31, 8'h58, 8'h52
    };

    localparam [103:0] INIT_DBG_1 = {
        8'h20, 8'h2D, 8'h2D, 8'h2D, 8'h2D, 8'h2D, 8'h2D, 8'h2D,
        8'h2D, 8'h3A, 8'h32, 8'h58, 8'h52
    };

    localparam [103:0] INIT_DBG_2 = {
        8'h2D, 8'h2D, 8'h2E, 8'h2D, 8'h2D, 8'h20, 8'h20, 8'h3A,
        8'h20, 8'h4C, 8'h4C, 8'h4F, 8'h52
    };

    localparam [103:0] INIT_DBG_3 = {
        8'h30, 8'h30, 8'h30, 8'h30, 8'h3A, 8'h45, 8'h20,
        8'h30, 8'h30, 8'h30, 8'h30, 8'h3A, 8'h4F
    };

    wire [COLS*8*4-1:0] TXT_INIT = OSD_DBG_EN
        ? { place_line(INIT_DBG_3), place_line(INIT_DBG_2),
            place_line(INIT_DBG_1), place_line(INIT_DBG_0) }
        : { PL_BLANK,               place_line(INIT_OFF_2),
            place_line(INIT_OFF_1), place_line(INIT_OFF_0) };

    reg [15:0] ar, ap, ay;
    reg        nr, np, ny;
    reg        dv1;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        ar  <= 16'd0; ap  <= 16'd0; ay  <= 16'd0;
        nr  <= 1'b0;  np  <= 1'b0;  ny  <= 1'b0;
        dv1 <= 1'b0;
    end
    else begin
        ar  <= RollRaw[15]  ? (~RollRaw  + 16'd1) : RollRaw;
        ap  <= PitchRaw[15] ? (~PitchRaw + 16'd1) : PitchRaw;
        ay  <= YawRaw[15]   ? (~YawRaw   + 16'd1) : YawRaw;
        nr  <= RollRaw[15];
        np  <= PitchRaw[15];
        ny  <= YawRaw[15];
        dv1 <= AngValid;
    end

    wire [31:0] pr_r = ar * 16'd18000;    
    wire [31:0] pr_p = ap * 16'd18000;
    wire [31:0] pr_y = ay * 16'd18000;

    reg [14:0] mr, mp, my;
    reg        nr2, np2, ny2;
    reg        link2;
    reg        dv2;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        mr <= 15'd0; mp <= 15'd0; my <= 15'd0;
        nr2 <= 1'b0; np2 <= 1'b0; ny2 <= 1'b0;
        link2 <= 1'b0; dv2 <= 1'b0;
    end
    else begin

        mr    <= pr_r[29:15];
        mp    <= pr_p[29:15];
        my    <= pr_y[29:15];
        nr2   <= nr;
        np2   <= np;
        ny2   <= ny;
        link2 <= LinkOk;
        dv2   <= dv1;
    end

    reg        fmt_v_a, fmt_v_b;
    reg [14:0] fmt_i_r_a, fmt_i_p_a, fmt_i_y_a;
    reg [14:0] fmt_mag_r_a, fmt_mag_p_a, fmt_mag_y_a;
    reg        fmt_nr_a, fmt_np_a, fmt_ny_a, fmt_link_a;
    reg [15:0] fmt_ok_a, fmt_err_a;
    reg [63:0] fmt_bytes_a;

    reg [14:0] fmt_i_r_b, fmt_i_p_b, fmt_i_y_b;
    reg [14:0] fmt_d_r_b, fmt_d_p_b, fmt_d_y_b;
    reg        fmt_nr_b, fmt_np_b, fmt_ny_b, fmt_link_b;
    reg [15:0] fmt_ok_b, fmt_err_b;
    reg [63:0] fmt_bytes_b;

    wire [31:0] fmt_pr_r = {17'd0, mr} * 32'd41944;
    wire [31:0] fmt_pr_p = {17'd0, mp} * 32'd41944;
    wire [31:0] fmt_pr_y = {17'd0, my} * 32'd41944;
    wire [29:0] fmt_t100_r = {15'd0, fmt_i_r_a} * 30'd100;
    wire [29:0] fmt_t100_p = {15'd0, fmt_i_p_a} * 30'd100;
    wire [29:0] fmt_t100_y = {15'd0, fmt_i_y_a} * 30'd100;
    wire [29:0] fmt_rem_r = {15'd0, fmt_mag_r_a} - fmt_t100_r;
    wire [29:0] fmt_rem_p = {15'd0, fmt_mag_p_a} - fmt_t100_p;
    wire [29:0] fmt_rem_y = {15'd0, fmt_mag_y_a} - fmt_t100_y;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        fmt_v_a <= 1'b0; fmt_v_b <= 1'b0;
        fmt_i_r_a <= 15'd0; fmt_i_p_a <= 15'd0; fmt_i_y_a <= 15'd0;
        fmt_mag_r_a <= 15'd0; fmt_mag_p_a <= 15'd0; fmt_mag_y_a <= 15'd0;
        fmt_nr_a <= 1'b0; fmt_np_a <= 1'b0; fmt_ny_a <= 1'b0; fmt_link_a <= 1'b0;
        fmt_ok_a <= 16'd0; fmt_err_a <= 16'd0; fmt_bytes_a <= 64'd0;
        fmt_i_r_b <= 15'd0; fmt_i_p_b <= 15'd0; fmt_i_y_b <= 15'd0;
        fmt_d_r_b <= 15'd0; fmt_d_p_b <= 15'd0; fmt_d_y_b <= 15'd0;
        fmt_nr_b <= 1'b0; fmt_np_b <= 1'b0; fmt_ny_b <= 1'b0; fmt_link_b <= 1'b0;
        fmt_ok_b <= 16'd0; fmt_err_b <= 16'd0; fmt_bytes_b <= 64'd0;
    end else begin
        fmt_v_a <= dv2;
        fmt_v_b <= fmt_v_a;
        if(dv2) begin
            fmt_i_r_a <= fmt_pr_r[31:22]; fmt_i_p_a <= fmt_pr_p[31:22]; fmt_i_y_a <= fmt_pr_y[31:22];
            fmt_mag_r_a <= mr; fmt_mag_p_a <= mp; fmt_mag_y_a <= my;
            fmt_nr_a <= nr2; fmt_np_a <= np2; fmt_ny_a <= ny2; fmt_link_a <= link2;
            fmt_ok_a <= DbgOk; fmt_err_a <= DbgErr; fmt_bytes_a <= DbgBytes;
        end
        if(fmt_v_a) begin
            fmt_i_r_b <= fmt_i_r_a; fmt_i_p_b <= fmt_i_p_a; fmt_i_y_b <= fmt_i_y_a;
            fmt_d_r_b <= fmt_rem_r[14:0]; fmt_d_p_b <= fmt_rem_p[14:0]; fmt_d_y_b <= fmt_rem_y[14:0];
            fmt_nr_b <= fmt_nr_a; fmt_np_b <= fmt_np_a; fmt_ny_b <= fmt_ny_a; fmt_link_b <= fmt_link_a;
            fmt_ok_b <= fmt_ok_a; fmt_err_b <= fmt_err_a; fmt_bytes_b <= fmt_bytes_a;
        end
    end

    function [103:0] fmt_line_parts;
        input [7:0]  c0, c1, c2, c3, c4, c5;
        input        link;
        input        neg;
        input [14:0] di;                         
        input [14:0] dr;                         
        reg   [7:0]  sgn, dh, dt, dob, dot, dv1b, dv0b;
        reg   [31:0] q1, q2;
        reg   [7:0]  ta, hu, te, ob, d1, d0;
        begin
            if(!link) begin
                sgn = 8'h20; dh = 8'h20; dt = 8'h2D; dob = 8'h2D;
                dot = 8'h2E; dv1b = 8'h2D; dv0b = 8'h2D;
            end else begin
                q1   = di * 16'd820;
                ta   = {3'b000, q1[17:13]};
                ob   = di[7:0] - ta * 8'd10;
                hu   = (ta >= 8'd10) ? 8'd1 : 8'd0;
                te   = ta - hu * 8'd10;
                q2   = dr * 16'd820;
                d1   = {3'b000, q2[17:13]};
                d0   = dr[7:0] - d1 * 8'd10;
                sgn  = neg ? 8'h2D : 8'h2B;
                dh   = 8'h30 + hu;
                dt   = 8'h30 + te;
                dob  = 8'h30 + ob;
                dot  = 8'h2E;
                dv1b = 8'h30 + d1;
                dv0b = 8'h30 + d0;
            end
            fmt_line_parts = { dv0b, dv1b, dot, dob, dt, dh, sgn,
                               c5, c4, c3, c2, c1, c0 };
        end
    endfunction

    function [7:0] hexch;
        input [3:0] n;
        begin
            hexch = (n < 4'd10) ? (8'h30 + {4'd0, n})           
                                : (8'h41 + {4'd0, n} - 8'd10);  
        end
    endfunction

    function [103:0] fmt_dbg;
        input [15:0] va, vb;
        begin

            fmt_dbg = { hexch(vb[3:0]),  hexch(vb[7:4]),
                        hexch(vb[11:8]), hexch(vb[15:12]),
                        8'h3A, 8'h45, 8'h20,                     
                        hexch(va[3:0]),  hexch(va[7:4]),
                        hexch(va[11:8]), hexch(va[15:12]),
                        8'h3A, 8'h4F };                          
        end
    endfunction

    function [103:0] fmt_bytes;
        input [31:0] b;              
        input [7:0]  c0, c1, c2;     
        begin
            fmt_bytes = { 8'h20,                               
                          hexch(b[3:0]),   hexch(b[7:4]),       
                          hexch(b[11:8]),  hexch(b[15:12]),     
                          hexch(b[19:16]), hexch(b[23:20]),     
                          hexch(b[27:24]), hexch(b[31:28]),     
                          8'h3A, c2, c1, c0 };                  
        end
    endfunction

    wire [103:0] ln_roll  = fmt_line_parts(8'h52, 8'h4F, 8'h4C, 8'h4C, 8'h20, 8'h3A,
                                            fmt_link_b, fmt_nr_b, fmt_i_r_b, fmt_d_r_b);
    wire [103:0] ln_pitch = fmt_line_parts(8'h50, 8'h49, 8'h54, 8'h43, 8'h48, 8'h3A,
                                            fmt_link_b, fmt_np_b, fmt_i_p_b, fmt_d_p_b);
    wire [103:0] ln_yaw   = fmt_line_parts(8'h59, 8'h41, 8'h57, 8'h20, 8'h20, 8'h3A,
                                            fmt_link_b, fmt_ny_b, fmt_i_y_b, fmt_d_y_b);

    wire [103:0] ln_dbg_rx1 = fmt_bytes(fmt_bytes_b[63:32], 8'h52, 8'h58, 8'h31);
    wire [103:0] ln_dbg_rx2 = fmt_bytes(fmt_bytes_b[31:0],  8'h52, 8'h58, 8'h32);
    wire [103:0] ln_dbg_cnt = fmt_dbg(fmt_ok_b, fmt_err_b);

    wire [COLS*8-1:0] pl_roll  = place_line(ln_roll);
    wire [COLS*8-1:0] pl_pitch = place_line(ln_pitch);
    wire [COLS*8-1:0] pl_yaw   = place_line(ln_yaw);
    wire [COLS*8-1:0] pl_rx1   = place_line(ln_dbg_rx1);
    wire [COLS*8-1:0] pl_rx2   = place_line(ln_dbg_rx2);
    wire [COLS*8-1:0] pl_cnt   = place_line(ln_dbg_cnt);

    wire [COLS*8*4-1:0] txt_next = OSD_DBG_EN
        ? { pl_cnt,   pl_roll,  pl_rx2, pl_rx1 }     
        : { PL_BLANK, pl_yaw,   pl_pitch, pl_roll };  

    reg wr_bank;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        TxtA    <= TXT_INIT;
        TxtB    <= TXT_INIT;
        wr_bank <= 1'b0;
        BankSel <= 1'b0;
    end
    else if(fmt_v_b) begin
        if(!wr_bank) TxtA <= txt_next;
        else         TxtB <= txt_next;
        BankSel <= wr_bank;       
        wr_bank <= ~wr_bank;
    end

endmodule
