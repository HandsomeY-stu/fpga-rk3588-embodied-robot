/*
 * jy901_rx.v
 * JY901姿态串口帧解析，校验帧头/校验和并发布新姿态。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
module uart_rx_8n1 #(
    parameter CLK_FREQ = 50_000_000,
    parameter BAUD     = 9600
)(
    input            Clk,
    input            Rst_n,
    input            Rx,
    output reg [7:0] Data,
    output reg       Valid       
);
    localparam integer DIV  = CLK_FREQ / BAUD;    
    localparam integer HALF = DIV / 2;            

    reg [1:0] rx_sync;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) rx_sync <= 2'b11;
    else       rx_sync <= {rx_sync[0], Rx};
    wire rx_d = rx_sync[1];

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [12:0] cnt;
    reg [2:0]  bidx;
    reg [7:0]  sh;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        state <= S_IDLE;
        cnt   <= 13'd0;
        bidx  <= 3'd0;
        sh    <= 8'd0;
        Data  <= 8'd0;
        Valid <= 1'b0;
    end
    else begin
        Valid <= 1'b0;
        case(state)

        S_IDLE: begin
            cnt <= 13'd0;
            if(!rx_d) state <= S_START;
        end

        S_START: begin
            if(cnt == HALF - 1) begin
                cnt <= 13'd0;
                if(!rx_d) begin bidx <= 3'd0; state <= S_DATA; end
                else            state <= S_IDLE;
            end
            else cnt <= cnt + 1'b1;
        end

        S_DATA: begin
            if(cnt == DIV - 1) begin
                cnt <= 13'd0;
                sh  <= {rx_d, sh[7:1]};           
                if(bidx == 3'd7) state <= S_STOP;
                else             bidx  <= bidx + 1'b1;
            end
            else cnt <= cnt + 1'b1;
        end

        S_STOP: begin
            if(cnt == DIV - 1) begin
                cnt   <= 13'd0;
                Data  <= sh;
                Valid <= 1'b1;
                state <= S_IDLE;
            end
            else cnt <= cnt + 1'b1;
        end
        default: state <= S_IDLE;
        endcase
    end
endmodule

module jy901_parse (
    input                    Clk,         
    input                    Rst_n,
    input      [7:0]         RxData,
    input                    RxValid,     
    output reg signed [15:0] Roll,
    output reg signed [15:0] Pitch,
    output reg signed [15:0] Yaw,
    output reg               AngValid,    
    output reg               LinkOk,      
    output reg [15:0]        OkCnt,       
    output reg [15:0]        ErrCnt,      
    output reg [63:0]        DbgBytes     
);
    reg [79:0] sr;                        
    reg [3:0]  bcnt;                      

    wire [7:0] b0 = sr[79:72];
    wire [7:0] b1 = sr[71:64];
    wire [7:0] b2 = sr[63:56];            
    wire [7:0] b3 = sr[55:48];            
    wire [7:0] b4 = sr[47:40];            
    wire [7:0] b5 = sr[39:32];            
    wire [7:0] b6 = sr[31:24];            
    wire [7:0] b7 = sr[23:16];            
    wire [7:0] b8 = sr[15:8];
    wire [7:0] b9 = sr[7:0];

    reg  [7:0] sum;
    always @(*) sum = b0 + b1 + b2 + b3 + b4 + b5 + b6 + b7 + b8 + b9;

    wire csum_ok  = (sum == RxData);                           
    wire type_ok  = (RxData >= 8'h50) && (RxData <= 8'h5A);    
    wire is_angle = (b1 == 8'h53);                             

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        sr       <= 80'd0;
        bcnt     <= 4'd0;
        Roll     <= 16'sd0;
        Pitch    <= 16'sd0;
        Yaw      <= 16'sd0;
        AngValid <= 1'b0;
        LinkOk   <= 1'b0;
        OkCnt    <= 16'd0;
        ErrCnt   <= 16'd0;
        DbgBytes <= 64'd0;
    end
    else begin
        AngValid <= 1'b0;
        if(RxValid) begin
            case(bcnt)

            4'd0: begin
                if(RxData == 8'h55) begin
                    sr   <= {sr[71:0], RxData};
                    bcnt <= 4'd1;
                end
            end

            4'd1: begin
                if(type_ok) begin
                    sr   <= {sr[71:0], RxData};
                    bcnt <= 4'd2;
                end
                else if(RxData == 8'h55) begin    
                    sr   <= {sr[71:0], RxData};
                    bcnt <= 4'd1;
                end
                else begin
                    bcnt <= 4'd0;
                end
            end

            default: begin
                if(bcnt < 4'd10) begin
                    sr   <= {sr[71:0], RxData};
                    bcnt <= bcnt + 1'b1;
                end

                else begin

                    DbgBytes <= sr[79:16];
                    if(csum_ok) begin
                        if(is_angle) begin
                            Roll     <= {b3, b2};     
                            Pitch    <= {b5, b4};
                            Yaw      <= {b7, b6};
                            AngValid <= 1'b1;
                            LinkOk   <= 1'b1;
                            OkCnt    <= OkCnt + 1'b1;
                        end
                        bcnt <= 4'd0;
                    end
                    else begin
                        ErrCnt <= ErrCnt + 1'b1;
                        if(RxData == 8'h55) begin
                            sr   <= {sr[71:0], RxData};
                            bcnt <= 4'd1;
                        end
                        else bcnt <= 4'd0;
                    end
                end
            end
            endcase
        end
    end
endmodule
