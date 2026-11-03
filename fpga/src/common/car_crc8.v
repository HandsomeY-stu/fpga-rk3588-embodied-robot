/*
 * car_crc8.v
 * CRC-8校验模块，统一串口协议的数据完整性检查。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
module car_crc8 (
    input             Clk,
    input             Rst_n,
    input             Clr,         
    input      [7:0]  Data,        
    input             DataValid,   
    output reg [7:0]  Crc          
);

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

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n)          Crc <= 8'h00;
    else if(Clr)        Crc <= 8'h00;
    else if(DataValid)  Crc <= crc8_byte(Crc, Data);
    else                Crc <= Crc;

endmodule
