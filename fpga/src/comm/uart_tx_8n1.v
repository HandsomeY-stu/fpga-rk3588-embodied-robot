/*
 * uart_tx_8n1.v
 * 参数化UART发送器，采用8数据位、无校验、1停止位。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
module uart_tx_8n1 #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 115200
)(
    input        Clk,
    input        Rst_n,

    input  [7:0] TxData,      
    input        TxValid,     
    output reg   Tx,          
    output reg   Busy,        
    output reg   TxDone       
);

    localparam integer DIV = CLK_FREQ / BAUD;    

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] cnt;
    reg [2:0]  bidx;
    reg [7:0]  sh;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        state  <= S_IDLE;
        cnt    <= 16'd0;
        bidx   <= 3'd0;
        sh     <= 8'd0;
        Tx     <= 1'b1;       
        Busy   <= 1'b0;
        TxDone <= 1'b0;
    end
    else begin
        TxDone <= 1'b0;

        case(state)

        S_IDLE: begin
            Tx   <= 1'b1;
            Busy <= 1'b0;
            cnt  <= 16'd0;
            if(TxValid) begin
                sh    <= TxData;
                Busy  <= 1'b1;
                state <= S_START;
            end
        end

        S_START: begin
            Tx <= 1'b0;                        
            if(cnt >= DIV - 1) begin
                cnt   <= 16'd0;
                bidx  <= 3'd0;
                state <= S_DATA;
            end
            else cnt <= cnt + 1'b1;
        end

        S_DATA: begin
            Tx <= sh[0];                       
            if(cnt >= DIV - 1) begin
                cnt <= 16'd0;
                sh  <= {1'b0, sh[7:1]};
                if(bidx == 3'd7) state <= S_STOP;
                else             bidx  <= bidx + 1'b1;
            end
            else cnt <= cnt + 1'b1;
        end

        S_STOP: begin
            Tx <= 1'b1;                        
            if(cnt >= DIV - 1) begin
                cnt    <= 16'd0;
                TxDone <= 1'b1;
                Busy   <= 1'b0;
                state  <= S_IDLE;
            end
            else cnt <= cnt + 1'b1;
        end

        default: state <= S_IDLE;
        endcase
    end

endmodule
