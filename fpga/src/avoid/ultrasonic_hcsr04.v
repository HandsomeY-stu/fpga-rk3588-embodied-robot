/*
 * ultrasonic_hcsr04.v
 * HC-SR04触发与回波计时接口，完成距离转换和超时处理。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module ultrasonic_hcsr04 #(
    parameter CLK_FREQ        = `CLK_CTRL_HZ,
    parameter TRIG_PERIOD_MS  = `US_TRIG_PERIOD_MS,
    parameter TIMEOUT_MS      = `US_TIMEOUT_MS,
    parameter MIN_MM          = `US_MIN_MM,
    parameter MAX_MM          = `US_MAX_MM,
    parameter FreeRun         = 1           
)(
    input                    Clk,
    input                    Rst_n,

    input                    Start,         
    output reg               Trig,          
    input                    Echo,          

    output reg [15:0]        Distance,      
    output reg               Valid,         
    output reg               Busy,          
    output reg [7:0]         TimeoutCnt     
);

    localparam integer CNT_PER_US    = CLK_FREQ / 1_000_000;              
    localparam integer TRIG_W_CNT    = CNT_PER_US * 10;                   
    localparam integer TIMEOUT_CNT   = CNT_PER_US * TIMEOUT_MS * 1000;    
    localparam integer PERIOD_CNT    = (CLK_FREQ / 1000) * TRIG_PERIOD_MS; 
    localparam integer MIN_CNT       = CNT_PER_US * 100;                  

    localparam [47:0] MM_Q20 = (48'd170 * 48'd1048576) / (48'd1000 * CNT_PER_US);

    localparam integer TMO_W  = 22;     
    localparam integer PER_W  = 23;     
    localparam integer ECH_W  = 22;     

    reg [1:0] echo_sync;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) echo_sync <= 2'b00;
    else       echo_sync <= {echo_sync[0], Echo};

    wire echo_d = echo_sync[1];

    reg echo_d1;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) echo_d1 <= 1'b0;
    else       echo_d1 <= echo_d;

    wire echo_rise =  echo_d & ~echo_d1;
    wire echo_fall = ~echo_d &  echo_d1;

    reg [47:0] mm_pr;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) mm_pr <= 48'd0;
    else       mm_pr <= {26'd0, ech_cnt} * MM_Q20;

    wire [47:0] mm_round = mm_pr + 48'd524288;    
    wire [15:0] mm_raw   = mm_round[35:20];       
    wire [15:0] mm_calc  = (mm_raw < MIN_MM) ? `US_INVALID :
                           (mm_raw > MAX_MM) ? `US_INVALID : mm_raw;

    localparam S_IDLE    = 3'd0,    
               S_TRIG    = 3'd1,    
               S_WAITECHO= 3'd2,    
               S_MEASURE = 3'd3,    
               S_DONE    = 3'd4;    

    reg [2:0]  state;
    reg [PER_W-1:0] per_cnt;      
    reg [15:0]      trig_cnt;     
    reg [TMO_W-1:0] tmo_cnt;      
    reg [ECH_W-1:0] ech_cnt;      

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        state      <= S_IDLE;
        per_cnt    <= {PER_W{1'b0}};
        trig_cnt   <= 16'd0;
        tmo_cnt    <= {TMO_W{1'b0}};
        ech_cnt    <= {ECH_W{1'b0}};
        Trig       <= 1'b0;
        Distance   <= `US_INVALID;
        Valid      <= 1'b0;
        Busy       <= 1'b0;
        TimeoutCnt <= 8'd0;
    end
    else begin
        Valid <= 1'b0;       

        case(state)

        S_IDLE: begin
            Trig  <= 1'b0;
            Busy  <= 1'b0;
            trig_cnt <= 16'd0;

            if(FreeRun) begin
                if(per_cnt >= PERIOD_CNT - 1) begin
                    per_cnt <= {PER_W{1'b0}};
                    state   <= S_TRIG;
                end
                else per_cnt <= per_cnt + 1'b1;
            end
            else begin
                if(Start) begin
                    per_cnt <= {PER_W{1'b0}};
                    state   <= S_TRIG;
                end
            end
        end

        S_TRIG: begin
            Busy <= 1'b1;
            Trig <= 1'b1;
            if(trig_cnt >= TRIG_W_CNT - 1) begin
                trig_cnt <= 16'd0;
                Trig     <= 1'b0;
                tmo_cnt  <= {TMO_W{1'b0}};
                state    <= S_WAITECHO;
            end
            else trig_cnt <= trig_cnt + 1'b1;
        end

        S_WAITECHO: begin
            if(echo_rise) begin
                ech_cnt <= {ECH_W{1'b0}};
                state   <= S_MEASURE;
            end
            else if(tmo_cnt >= TIMEOUT_CNT - 1) begin
                tmo_cnt    <= {TMO_W{1'b0}};
                Distance   <= `US_INVALID;
                TimeoutCnt <= TimeoutCnt + 1'b1;
                Valid      <= 1'b1;
                state      <= S_IDLE;
            end
            else tmo_cnt <= tmo_cnt + 1'b1;
        end

        S_MEASURE: begin
            if(echo_fall) begin
                state <= S_DONE;
            end
            else if(ech_cnt >= TIMEOUT_CNT - 1) begin    
                ech_cnt    <= {ECH_W{1'b0}};
                Distance   <= `US_INVALID;
                TimeoutCnt <= TimeoutCnt + 1'b1;
                Valid      <= 1'b1;
                state      <= S_IDLE;
            end
            else ech_cnt <= ech_cnt + 1'b1;
        end

        S_DONE: begin

            if(ech_cnt < MIN_CNT) begin
                Distance <= `US_INVALID;
                Valid    <= 1'b1;
            end
            else begin
                Distance <= mm_calc;
                Valid    <= 1'b1;
            end
            state <= S_IDLE;
        end

        default: state <= S_IDLE;
        endcase
    end

endmodule
