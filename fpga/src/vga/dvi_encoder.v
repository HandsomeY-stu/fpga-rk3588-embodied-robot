/*
 * dvi_encoder.v
 * RGB与同步信号的TMDS编码及串行输出。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`timescale 1 ps / 1ps
`define D #1

module dvi_encoder(
	input       pixelclk,        
	input       pixelclk5x,      
	input       rst_n,           
	input[7:0]  blue_din,        
	input[7:0]  green_din,       
	input[7:0]  red_din,         
	input       hsync,           
	input       vsync,           
	input       de,              
	output      tmds_clk_p,
	output      tmds_clk_n,
	output[2:0] tmds_data_p,     
	output[2:0] tmds_data_n      
);

	wire [9:0] red;
	wire [9:0] green;
	wire [9:0] blue;

	encode encb(
		.clk(pixelclk),
		.rst_n(rst_n),
		.din(blue_din),
		.c0(hsync),
		.c1(vsync),
		.de(de),
		.dout(blue)
	);

	encode encg(
		.clk(pixelclk),
		.rst_n(rst_n),
		.din(green_din),
		.c0(1'b0),
		.c1(1'b0),
		.de(de),
		.dout(green)
	);

	encode encr(
		.clk(pixelclk),
		.rst_n(rst_n),
		.din(red_din),
		.c0(1'b0),
		.c1(1'b0),
		.de(de),
		.dout(red)
	);

	oser_4b_10to1 serdes_4b_10to1_inst(
		.pixelclk(pixelclk),
		.rst_n(rst_n),
		.clkx5(pixelclk5x), 
		.datain_0(blue), 
		.datain_1(green), 
		.datain_2(red), 
		.datain_3(10'b1111100000), 
		.dataout_0_p(tmds_data_p[0]), 
		.dataout_0_n(tmds_data_n[0]), 
		.dataout_1_p(tmds_data_p[1]), 
		.dataout_1_n(tmds_data_n[1]), 
		.dataout_2_p(tmds_data_p[2]), 
		.dataout_2_n(tmds_data_n[2]), 
		.dataout_3_p(tmds_clk_p), 
		.dataout_3_n(tmds_clk_n)  
	); 

endmodule

module encode(
	clk,
	rst_n,
	din,
	c0,
	c1,
	de,
	dout
);
	input            clk;     
	input            rst_n;     
	input      [7:0] din;       
	input            c0;        
	input            c1;        
	input            de;        
	output reg [9:0] dout;      

	parameter CTL0 = 10'b1101010100;
	parameter CTL1 = 10'b0010101011;
	parameter CTL2 = 10'b0101010100;
	parameter CTL3 = 10'b1010101011;

	reg [3:0] n1d; 		 
	reg [7:0] din_q;	 

	always @(posedge clk) begin
		din_q <= `D din;
		n1d <= `D din[0] + din[1] + din[2] + din[3] + din[4] + din[5] + din[6] + din[7];
	end

	wire decision1;	 
	assign decision1 =(n1d > 4'h4) |((n1d == 4'h4) &(din_q[0] == 1'b0));

	wire [8:0] q_m;
	assign q_m[0] = din_q[0];
	assign q_m[1] =(decision1) ? ~(q_m[0] ^ din_q[1]) :(q_m[0] ^ din_q[1]);
	assign q_m[2] =(decision1) ? ~(q_m[1] ^ din_q[2]) :(q_m[1] ^ din_q[2]);
	assign q_m[3] =(decision1) ? ~(q_m[2] ^ din_q[3]) :(q_m[2] ^ din_q[3]);
	assign q_m[4] =(decision1) ? ~(q_m[3] ^ din_q[4]) :(q_m[3] ^ din_q[4]);
	assign q_m[5] =(decision1) ? ~(q_m[4] ^ din_q[5]) :(q_m[4] ^ din_q[5]);
	assign q_m[6] =(decision1) ? ~(q_m[5] ^ din_q[6]) :(q_m[5] ^ din_q[6]);
	assign q_m[7] =(decision1) ? ~(q_m[6] ^ din_q[7]) :(q_m[6] ^ din_q[7]);
	assign q_m[8] =(decision1) ? 1'b0 : 1'b1;

	reg [3:0] n1q_m, n0q_m;  
	always @(posedge clk) begin
		n1q_m  <= `D q_m[0] + q_m[1] + q_m[2] + q_m[3] + q_m[4] + q_m[5] + q_m[6] + q_m[7];
		n0q_m  <= `D 4'h8 -(q_m[0] + q_m[1] + q_m[2] + q_m[3] + q_m[4] + q_m[5] + q_m[6] + q_m[7]);
	end

	reg [4:0] cnt;  
	wire decision2, decision3;
	assign decision2 =(cnt == 5'h0) |(n1q_m == n0q_m);

	assign decision3 =(~cnt[4] &(n1q_m > n0q_m)) |(cnt[4] &(n0q_m > n1q_m));

	reg [1:0] de_reg;
	reg [1:0] c0_reg;
	reg [1:0] c1_reg;
	reg [8:0] q_m_reg;
	always @(posedge clk) begin
		de_reg  <= `D {de_reg[0], de};
		c0_reg  <= `D {c0_reg[0], c0};
		c1_reg  <= `D {c1_reg[0], c1};
		q_m_reg <= `D q_m;
	end

	always @(posedge clk or negedge rst_n) begin
		if(!rst_n) begin
			dout <= 10'h0;
			cnt <= 5'd0;
		end else begin
			if(de_reg[1]) begin 
				if(decision2) begin
					dout[9]   <= `D ~q_m_reg[8]; 
					dout[8]   <= `D q_m_reg[8]; 
					dout[7:0] <= `D(q_m_reg[8]) ? q_m_reg[7:0] : ~q_m_reg[7:0];
					cnt <= `D(~q_m_reg[8]) ?(cnt + n0q_m - n1q_m) :(cnt + n1q_m - n0q_m);
				end else begin if(decision3) begin
						dout[9]   <= `D 1'b1;
						dout[8]   <= `D q_m_reg[8];
						dout[7:0] <= `D ~q_m_reg;
						cnt <= `D cnt + {q_m_reg[8], 1'b0} +(n0q_m - n1q_m);
					end else begin
						dout[9]   <= `D 1'b0;
						dout[8]   <= `D q_m_reg[8];
						dout[7:0] <= `D q_m_reg[7:0];
						cnt <= `D cnt - {~q_m_reg[8], 1'b0} +(n1q_m - n0q_m);
					end
				end
			end else begin	 
				cnt <=  `D 5'd0;
				case({c1_reg[1], c0_reg[1]})
					2'b00:   dout <= `D CTL0;
					2'b01:   dout <= `D CTL1;
					2'b10:   dout <= `D CTL2;
					default: dout <= `D CTL3;
				endcase
			end
		end
	end

endmodule

module serdes_4b_10to1(
	input          clkx5,          
	input [9:0]    datain_0,       
	input [9:0]    datain_1,       
	input [9:0]    datain_2,       
	input [9:0]    datain_3,       
	output         dataout_0_p,    
	output         dataout_0_n,    
	output         dataout_1_p,    
	output         dataout_1_n,    
	output         dataout_2_p,    
	output         dataout_2_n,    
	output         dataout_3_p,    
	output         dataout_3_n     
);  

	reg [2:0] TMDS_mod5 = 0;   

	reg [4:0] TMDS_shift_0h = 0, TMDS_shift_0l = 0;
	reg [4:0] TMDS_shift_1h = 0, TMDS_shift_1l = 0;
	reg [4:0] TMDS_shift_2h = 0, TMDS_shift_2l = 0;
	reg [4:0] TMDS_shift_3h = 0, TMDS_shift_3l = 0;

	wire [4:0] TMDS_0_l = {datain_0[9],datain_0[7],datain_0[5],datain_0[3],datain_0[1]};
	wire [4:0] TMDS_0_h = {datain_0[8],datain_0[6],datain_0[4],datain_0[2],datain_0[0]};

	wire [4:0] TMDS_1_l = {datain_1[9],datain_1[7],datain_1[5],datain_1[3],datain_1[1]};
	wire [4:0] TMDS_1_h = {datain_1[8],datain_1[6],datain_1[4],datain_1[2],datain_1[0]};

	wire [4:0] TMDS_2_l = {datain_2[9],datain_2[7],datain_2[5],datain_2[3],datain_2[1]};
	wire [4:0] TMDS_2_h = {datain_2[8],datain_2[6],datain_2[4],datain_2[2],datain_2[0]};

	wire [4:0] TMDS_3_l = {datain_3[9],datain_3[7],datain_3[5],datain_3[3],datain_3[1]};
	wire [4:0] TMDS_3_h = {datain_3[8],datain_3[6],datain_3[4],datain_3[2],datain_3[0]};

	always @(posedge clkx5)
	begin
		if(TMDS_mod5 >= 3'd4)
			TMDS_mod5 <= 3'd0;
		else
			TMDS_mod5 <= TMDS_mod5 + 3'd1;
	end

	always @(posedge clkx5)
	begin
		if(TMDS_mod5 == 3'd4)begin
			TMDS_shift_0h <= TMDS_0_h;
			TMDS_shift_0l <= TMDS_0_l;
			TMDS_shift_1h <= TMDS_1_h;
			TMDS_shift_1l <= TMDS_1_l;
			TMDS_shift_2h <= TMDS_2_h;
			TMDS_shift_2l <= TMDS_2_l;	
			TMDS_shift_3h <= TMDS_3_h;
			TMDS_shift_3l <= TMDS_3_l;		
		end
		else begin
			TMDS_shift_0h <= TMDS_shift_0h[4:1];
			TMDS_shift_0l <= TMDS_shift_0l[4:1];
			TMDS_shift_1h <= TMDS_shift_1h[4:1];
			TMDS_shift_1l <= TMDS_shift_1l[4:1];
			TMDS_shift_2h <= TMDS_shift_2h[4:1];
			TMDS_shift_2l <= TMDS_shift_2l[4:1];	
			TMDS_shift_3h <= TMDS_shift_3h[4:1];
			TMDS_shift_3l <= TMDS_shift_3l[4:1];				
		end
	end

	Gowin_DDR Gowin_DDR_1(
		.din({TMDS_shift_3l[0],TMDS_shift_2l[0],TMDS_shift_1l[0],TMDS_shift_0l[0],TMDS_shift_3h[0],TMDS_shift_2h[0],TMDS_shift_1h[0],TMDS_shift_0h[0]}),  
		.clk(clkx5),  
		.q({dataout_3_p,dataout_2_p,dataout_1_p,dataout_0_p})  
	);

	Gowin_DDR Gowin_DDR_2(
		.din(~{TMDS_shift_3l[0],TMDS_shift_2l[0],TMDS_shift_1l[0],TMDS_shift_0l[0],TMDS_shift_3h[0],TMDS_shift_2h[0],TMDS_shift_1h[0],TMDS_shift_0h[0]}),  
		.clk(clkx5),  
		.q({dataout_3_n,dataout_2_n,dataout_1_n,dataout_0_n})  
	);

endmodule

module oser_4b_10to1(
	input         pixelclk,
	input         clkx5,
	input         rst_n,
	input  [9:0]  datain_0,
	input  [9:0]  datain_1,
	input  [9:0]  datain_2,
	input  [9:0]  datain_3,
	output        dataout_0_p, dataout_0_n,
	output        dataout_1_p, dataout_1_n,
	output        dataout_2_p, dataout_2_n,
	output        dataout_3_p, dataout_3_n
);
	wire rst = ~rst_n;
	wire ser_0, ser_1, ser_2, ser_3;

	OSER10 u_oser_0(.Q(ser_0), .PCLK(pixelclk), .FCLK(clkx5), .RESET(rst),
		.D0(datain_0[0]),.D1(datain_0[1]),.D2(datain_0[2]),.D3(datain_0[3]),.D4(datain_0[4]),
		.D5(datain_0[5]),.D6(datain_0[6]),.D7(datain_0[7]),.D8(datain_0[8]),.D9(datain_0[9]));
	OSER10 u_oser_1(.Q(ser_1), .PCLK(pixelclk), .FCLK(clkx5), .RESET(rst),
		.D0(datain_1[0]),.D1(datain_1[1]),.D2(datain_1[2]),.D3(datain_1[3]),.D4(datain_1[4]),
		.D5(datain_1[5]),.D6(datain_1[6]),.D7(datain_1[7]),.D8(datain_1[8]),.D9(datain_1[9]));
	OSER10 u_oser_2(.Q(ser_2), .PCLK(pixelclk), .FCLK(clkx5), .RESET(rst),
		.D0(datain_2[0]),.D1(datain_2[1]),.D2(datain_2[2]),.D3(datain_2[3]),.D4(datain_2[4]),
		.D5(datain_2[5]),.D6(datain_2[6]),.D7(datain_2[7]),.D8(datain_2[8]),.D9(datain_2[9]));
	OSER10 u_oser_3(.Q(ser_3), .PCLK(pixelclk), .FCLK(clkx5), .RESET(rst),
		.D0(datain_3[0]),.D1(datain_3[1]),.D2(datain_3[2]),.D3(datain_3[3]),.D4(datain_3[4]),
		.D5(datain_3[5]),.D6(datain_3[6]),.D7(datain_3[7]),.D8(datain_3[8]),.D9(datain_3[9]));

	ELVDS_OBUF u_obuf_0(.I(ser_0), .O(dataout_0_p), .OB(dataout_0_n));
	ELVDS_OBUF u_obuf_1(.I(ser_1), .O(dataout_1_p), .OB(dataout_1_n));
	ELVDS_OBUF u_obuf_2(.I(ser_2), .O(dataout_2_p), .OB(dataout_2_n));
	ELVDS_OBUF u_obuf_3(.I(ser_3), .O(dataout_3_p), .OB(dataout_3_n));
endmodule
