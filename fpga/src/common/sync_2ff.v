/*
 * sync_2ff.v
 * 单比特双触发器同步器，用于跨时钟域状态采样。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
module sync_2ff (
    input       Clk,         
    input       Rst_n,       
    input       AsyncIn,     
    output wire SyncOut      
);

    reg SyncMeta;    
    reg SyncReg;     

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        SyncMeta <= 1'b0;
        SyncReg  <= 1'b0;
    end
    else begin
        SyncMeta <= AsyncIn;
        SyncReg  <= SyncMeta;
    end

    assign SyncOut = SyncReg;

endmodule
