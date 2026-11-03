/*
 * us_poll.v
 * 多路超声波轮询调度，错开触发以降低通道相互干扰。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module us_poll #(
    parameter CLK_FREQ      = `CLK_CTRL_HZ,
    parameter N             = `US_NUM,           
    parameter GAP_MS        = 5,                 
    parameter WAIT_MS       = 40                 
)(
    input                       Clk,
    input                       Rst_n,

    input                       En,              
    input      [N-1:0]          ValidIn,         
    input      [N*16-1:0]       DistanceIn,      

    output reg [N-1:0]          Start,           
    output reg [N*16-1:0]       DistanceOut,     
    output reg [N-1:0]          TimeoutFlag,     
    output reg [N-1:0]          DoneMask,        
    output reg [15:0]           RoundCnt,        
    output reg [1:0]            CurIdx           
);

    localparam integer GAP_CNT  = (CLK_FREQ / 1000) * GAP_MS;
    localparam integer WAIT_CNT = (CLK_FREQ / 1000) * WAIT_MS;

    localparam S_IDLE = 2'd0,    
               S_PULSE= 2'd1,    
               S_WAIT = 2'd2,    
               S_GAP  = 2'd3;    

    reg [1:0]  state;
    reg [23:0] cnt;

    wire cur_valid = ValidIn[CurIdx];

    integer k;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        state       <= S_IDLE;
        cnt         <= 24'd0;
        Start       <= {N{1'b0}};
        CurIdx      <= 2'd0;
        DoneMask    <= {N{1'b0}};
        TimeoutFlag <= {N{1'b0}};
        RoundCnt    <= 16'd0;
        for(k = 0; k < N; k = k + 1) DistanceOut[k*16 +: 16] <= `US_INVALID;
    end
    else begin
        Start <= {N{1'b0}};           

        case(state)

        S_IDLE: begin
            CurIdx <= 2'd0;
            cnt    <= 24'd0;
            if(En) begin
                DoneMask <= {N{1'b0}};

                TimeoutFlag <= {N{1'b0}};
                state       <= S_PULSE;
            end
        end

        S_PULSE: begin
            Start[CurIdx] <= 1'b1;
            cnt           <= 24'd0;
            state         <= S_WAIT;
        end

        S_WAIT: begin
            if(cur_valid) begin

                DistanceOut[CurIdx*16 +: 16] <= DistanceIn[CurIdx*16 +: 16];

                TimeoutFlag[CurIdx] <= (DistanceIn[CurIdx*16 +: 16] == `US_INVALID);
                DoneMask[CurIdx]    <= 1'b1;
                cnt                 <= 24'd0;
                state               <= S_GAP;
            end
            else if(cnt >= WAIT_CNT - 1) begin

                DistanceOut[CurIdx*16 +: 16] <= `US_INVALID;
                TimeoutFlag[CurIdx]          <= 1'b1;
                DoneMask[CurIdx]             <= 1'b1;
                cnt                          <= 24'd0;
                state                        <= S_GAP;
            end
            else cnt <= cnt + 1'b1;
        end

        S_GAP: begin
            if(cnt >= GAP_CNT - 1) begin
                cnt <= 24'd0;
                if(CurIdx >= N - 1) begin

                    CurIdx      <= 2'd0;
                    RoundCnt    <= RoundCnt + 1'b1;
                    state       <= S_IDLE;
                end
                else begin
                    CurIdx <= CurIdx + 1'b1;
                    state  <= S_PULSE;
                end
            end
            else cnt <= cnt + 1'b1;
        end

        default: state <= S_IDLE;
        endcase
    end

endmodule
