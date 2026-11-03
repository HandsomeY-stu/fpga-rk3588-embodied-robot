/*
 * nav_fsm.v
 * 通用路点导航引擎；源码保留，当前集成默认采用任务四固定路线模式。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module nav_fsm #(
    parameter CLK_FREQ   = `CLK_CTRL_HZ,
    parameter ARRIVE_CM  = 10,        
    parameter GO_SPEED   = 8'd45,     
    parameter TURN_SPEED = 8'd35,     
    parameter STOP_MM    = `AVOID_STOP_MM,
    parameter BACK_MS    = 400,
    parameter TURN_MS    = 700
)(
    input                    Clk,
    input                    Rst_n,

    input                    En,           
    input                    Start,        
    input                    Abort,        
    input                    WpNew,        
    input                    Tick,         

    input      signed [15:0] WpX, WpY,     
    input      signed [15:0] CurX, CurY,   
    input      signed [15:0] CurYaw,       
    input      [15:0]        RangeF,       

    output reg [7:0]         State,        
    output reg [7:0]         WpIdx,        
    output reg [7:0]         DirOut,       
    output reg [7:0]         SpeedOut,     
    output reg               Active,       
    output     [15:0]        RemainCm      
);

    localparam S_IDLE    = `NAV_IDLE,
               S_TURN    = `NAV_TURN,
               S_GO      = `NAV_GO,
               S_AVOID   = `NAV_AVOID,
               S_ARRIVED = `NAV_ARRIVED,
               S_DONE    = `NAV_DONE;

    localparam integer BACK_CNT = (BACK_MS / `CTRL_PERIOD_MS);
    localparam integer TURN_CNT = (TURN_MS / `CTRL_PERIOD_MS);

    wire signed [16:0] dx = {WpX[15], WpX} - {CurX[15], CurX};
    wire signed [16:0] dy = {WpY[15], WpY} - {CurY[15], CurY};

    wire signed [16:0] yaw17 = {CurYaw[15], CurYaw};
    wire signed [15:0] sy, cy;

    sincos_lut u_sincos (
        .Ang (yaw17),
        .Sin (sy),
        .Cos (cy)
    );

    wire signed [47:0] p_dxcy = dx * cy;
    wire signed [47:0] p_dysy = dy * sy;
    wire signed [47:0] p_dxsy = dx * sy;
    wire signed [47:0] p_dycy = dy * cy;

    wire signed [47:0] ef_sum = p_dxcy + p_dysy;    
    wire signed [47:0] el_sum = p_dycy - p_dxsy;    

    wire signed [31:0] e_fwd  = ef_sum[46:15];      
    wire signed [31:0] e_left = el_sum[46:15];

    wire signed [31:0] el_abs = e_left[31] ? (~e_left + 32'sd1) : e_left;

    wire [16:0] dx_abs = dx[16] ? (~dx + 17'sd1) : dx;
    wire [16:0] dy_abs = dy[16] ? (~dy + 17'sd1) : dy;
    wire        arrived = (dx_abs <= ARRIVE_CM) && (dy_abs <= ARRIVE_CM);

    wire [16:0] rem_max = (dx_abs > dy_abs) ? dx_abs : dy_abs;
    assign RemainCm = (rem_max > 17'd32767) ? 16'h7FFF : rem_max[15:0];

    wire behind    = (e_fwd <= 32'sd0);               
    wire turn_hard = behind || (el_abs > e_fwd);                 
    wire turn_mid  = (el_abs > (e_fwd >>> 1));                   
    wire turn_soft = (el_abs > (e_fwd >>> 3));                   
    wire left_side = (e_left > 32'sd0);                          

    reg [15:0] hold_cnt;

    reg start_lat, wpnew_lat;

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        start_lat <= 1'b0;
        wpnew_lat <= 1'b0;
    end
    else if(Abort) begin
        start_lat <= 1'b0;
        wpnew_lat <= 1'b0;
    end
    else begin

        if(Start && (State == S_IDLE))  start_lat <= 1'b1;
        else if(Tick && (State == S_IDLE)) start_lat <= 1'b0;

        if(WpNew)                              wpnew_lat <= 1'b1;
        else if(Tick && (State == S_ARRIVED))  wpnew_lat <= 1'b0;
    end

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        State    <= S_IDLE;
        WpIdx    <= 8'd0;
        DirOut   <= `DIR_STOP;
        SpeedOut <= 8'd0;
        Active   <= 1'b0;
        hold_cnt <= 16'd0;
    end
    else if(Abort) begin
        State    <= S_IDLE;
        Active   <= 1'b0;
        DirOut   <= `DIR_STOP;
        SpeedOut <= 8'd0;
        hold_cnt <= 16'd0;
    end
    else if(!En) begin
        State    <= S_IDLE;
        Active   <= 1'b0;
        DirOut   <= `DIR_STOP;
        SpeedOut <= 8'd0;
        hold_cnt <= 16'd0;
    end
    else if(Tick) begin
        case(State)

        S_IDLE: begin
            Active   <= 1'b0;
            DirOut   <= `DIR_STOP;
            SpeedOut <= 8'd0;
            if(start_lat) begin           
                WpIdx  <= 8'd1;
                Active <= 1'b1;
                State  <= S_TURN;
            end
        end

        S_TURN: begin
            Active <= 1'b1;
            if(!turn_soft) begin
                State <= S_GO;                  
            end
            else begin

                DirOut   <= left_side ? `DIR_SPIN_L : `DIR_SPIN_R;
                SpeedOut <= TURN_SPEED;
            end
        end

        S_GO: begin
            Active <= 1'b1;

            if(arrived) begin
                State    <= S_ARRIVED;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else if((RangeF != `US_INVALID) && (RangeF <= STOP_MM)) begin

                State    <= S_AVOID;
                hold_cnt <= 16'd0;
            end
            else if(behind || turn_hard) begin

                State <= S_TURN;
            end
            else begin
                SpeedOut <= GO_SPEED;
                if(turn_mid)       DirOut <= left_side ? `DIR_LEFT  : `DIR_RIGHT;
                else if(turn_soft) DirOut <= left_side ? `DIR_LEFT  : `DIR_RIGHT;
                else               DirOut <= `DIR_FWD;
            end
        end

        S_AVOID: begin
            Active   <= 1'b1;
            hold_cnt <= hold_cnt + 1'b1;

            if(hold_cnt < BACK_CNT) begin
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
            else if(hold_cnt < BACK_CNT + TURN_CNT) begin
                DirOut   <= `DIR_BACK;
                SpeedOut <= 8'd40;
            end
            else if(hold_cnt < BACK_CNT + TURN_CNT + TURN_CNT) begin

                DirOut   <= `DIR_SPIN_L;
                SpeedOut <= TURN_SPEED;
            end
            else begin
                State    <= S_TURN;
                hold_cnt <= 16'd0;
                DirOut   <= `DIR_STOP;
                SpeedOut <= 8'd0;
            end
        end

        S_ARRIVED: begin
            DirOut   <= `DIR_STOP;
            SpeedOut <= 8'd0;
            if(wpnew_lat) begin           
                WpIdx <= WpIdx + 1'b1;
                State <= S_TURN;
            end
        end

        S_DONE: begin
            Active   <= 1'b0;
            DirOut   <= `DIR_STOP;
            SpeedOut <= 8'd0;
            State    <= S_IDLE;
        end

        default: State <= S_IDLE;
        endcase
    end

endmodule
