/*
 * sincos_lut.v
 * 角度到正余弦的定点查找表，为里程投影提供系数。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module sincos_lut (
    input      signed [16:0] Ang,      
    output     signed [15:0] Sin,      
    output     signed [15:0] Cos       
);

    function signed [15:0] sin_tab;
        input [6:0] j;
        reg signed [15:0] s;
        begin
            case(j)
            7'd0: s = 16'sd0;     7'd1: s = 16'sd732;   7'd2: s = 16'sd1464;  7'd3: s = 16'sd2194;
            7'd4: s = 16'sd2924;  7'd5: s = 16'sd3653;  7'd6: s = 16'sd4379;  7'd7: s = 16'sd5103;
            7'd8: s = 16'sd5825;  7'd9: s = 16'sd6544;  7'd10: s = 16'sd7259; 7'd11: s = 16'sd7971;
            7'd12: s = 16'sd8679; 7'd13: s = 16'sd9383; 7'd14: s = 16'sd10082;7'd15: s = 16'sd10776;
            7'd16: s = 16'sd11465;7'd17: s = 16'sd12147;7'd18: s = 16'sd12824;7'd19: s = 16'sd13495;
            7'd20: s = 16'sd14158;7'd21: s = 16'sd14815;7'd22: s = 16'sd15464;7'd23: s = 16'sd16105;
            7'd24: s = 16'sd16739;7'd25: s = 16'sd17364;7'd26: s = 16'sd17980;7'd27: s = 16'sd18588;
            7'd28: s = 16'sd19186;7'd29: s = 16'sd19774;7'd30: s = 16'sd20353;7'd31: s = 16'sd20922;
            7'd32: s = 16'sd21480;7'd33: s = 16'sd22027;7'd34: s = 16'sd22564;7'd35: s = 16'sd23089;
            7'd36: s = 16'sd23602;7'd37: s = 16'sd24104;7'd38: s = 16'sd24594;7'd39: s = 16'sd25072;
            7'd40: s = 16'sd25537;7'd41: s = 16'sd25989;7'd42: s = 16'sd26428;7'd43: s = 16'sd26854;
            7'd44: s = 16'sd27267;7'd45: s = 16'sd27666;7'd46: s = 16'sd28051;7'd47: s = 16'sd28423;
            7'd48: s = 16'sd28780;7'd49: s = 16'sd29123;7'd50: s = 16'sd29451;7'd51: s = 16'sd29764;
            7'd52: s = 16'sd30063;7'd53: s = 16'sd30347;7'd54: s = 16'sd30615;7'd55: s = 16'sd30868;
            7'd56: s = 16'sd31106;7'd57: s = 16'sd31329;7'd58: s = 16'sd31535;7'd59: s = 16'sd31726;
            7'd60: s = 16'sd31901;7'd61: s = 16'sd32060;7'd62: s = 16'sd32204;7'd63: s = 16'sd32331;
            7'd64: s = 16'sd32442;7'd65: s = 16'sd32537;7'd66: s = 16'sd32615;7'd67: s = 16'sd32677;
            7'd68: s = 16'sd32723;7'd69: s = 16'sd32753;7'd70: s = 16'sd32766;
            default: s = 16'sd32767;
            endcase
            sin_tab = s;
        end
    endfunction

    function signed [15:0] sin_q15;
        input signed [16:0] ang;
        reg [16:0] a, r, r2;
        reg [1:0]  q;
        reg signed [15:0] s;
        begin

            if(ang[16]) a = 17'd36000 + ang;
            else        a = ang[16:0];
            if(a >= 17'd36000) a = a - 17'd36000;

            if(a >= 17'd27000)      begin q = 2'd3; r = a - 17'd27000; end
            else if(a >= 17'd18000) begin q = 2'd2; r = a - 17'd18000; end
            else if(a >= 17'd9000)  begin q = 2'd1; r = a - 17'd9000;  end
            else                    begin q = 2'd0; r = a;              end

            r2 = q[0] ? (17'd9000 - r) : r;

            s = sin_tab(r2[13:7]);

            sin_q15 = q[1] ? -s : s;
        end
    endfunction

    assign Sin = sin_q15(Ang);
    assign Cos = sin_q15(Ang + 17'sd9000);    

endmodule
