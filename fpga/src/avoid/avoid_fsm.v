/*
 * avoid_fsm.v
 * 超声波避障状态机，协调减速、停车和绕障动作。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`include "car_defines.vh"

module avoid_fsm #(
    parameter CLK_FREQ   = `CLK_CTRL_HZ,
    parameter SLOW_MM    = `AVOID_SLOW_MM,
    parameter STOP_MM    = `AVOID_STOP_MM,
    parameter CLEAR_MM   = `AVOID_CLEAR_MM,
    parameter SLOW_SCALE = `AVOID_SLOW_SCALE,    

    parameter BACK_MS    = 400,
    parameter TURN_MS    = 1200,                 

    parameter STUCK_MS   = 900,                  
    parameter RESUME_MS  = 300,                  

    parameter BACK_SPEED = 8'd40,                

    parameter TURN_SPEED = 8'd30,

    parameter TURN_LEFT  = 1,                    
    parameter REAR_STOP_MM = 200                 
)(
    input                    Clk,
    input                    Rst_n,

    input                    En,           
    input                    Tick,         

    input      [15:0]        RangeF,       
    input      [15:0]        RangeRear,    

    input      [7:0]         CmdDir,       
    input      [7:0]         CmdSpeed,     

    output reg [7:0]         DirOut,       
    output reg [7:0]         SpeedOut,     
    output reg               AvoidActive,  
    output reg               Near,         
    output reg [7:0]         State         
);

    localparam S_NORMAL = 8'd0,    
               S_SLOW   = 8'd1,    
               S_STOP   = 8'd2,    
               S_BACKUP = 8'd3,    
               S_TURN   = 8'd4,    
               S_RESUME = 8'd5;    

    localparam integer STUCK_CNT  = (`CTRL_PERIOD_MS > 0) ? (STUCK_MS  / `CTRL_PERIOD_MS) : 1;
    localparam integer BACK_CNT   = (`CTRL_PERIOD_MS > 0) ? (BACK_MS   / `CTRL_PERIOD_MS) : 1;
    localparam integer TURN_CNT   = (`CTRL_PERIOD_MS > 0) ? (TURN_MS   / `CTRL_PERIOD_MS) : 1;
    localparam integer RESUME_CNT = (`CTRL_PERIOD_MS > 0) ? (RESUME_MS / `CTRL_PERIOD_MS) : 1;

    wire f_valid = (RangeF != `US_INVALID);
    wire f_stop  = f_valid && (RangeF <= STOP_MM);
    wire f_slow  = f_valid && (RangeF <= SLOW_MM);
    wire f_clear = (!f_valid) || (RangeF >= CLEAR_MM);

    wire       r_valid   = (RangeRear != `US_INVALID);
    wire       r_block   = r_valid && (RangeRear <= REAR_STOP_MM);   
    wire [7:0] turn_dir_w = TURN_LEFT ? `DIR_SPIN_L : `DIR_SPIN_R;   

    wire [47:0] cs_w = {32'd0, CmdSpeed};       

    localparam [47:0] SCALE_W  = SLOW_SCALE;
    localparam [47:0] K_DIV100 = 48'd41944;     

    reg [47:0] sl_pr;
    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) sl_pr <= 48'd0;
    else       sl_pr <= cs_w * SCALE_W * K_DIV100;

    wire [15:0] slow_spd = sl_pr[37:22];    

    reg [15:0] hold_cnt;      

    always @(posedge Clk or negedge Rst_n)
    if(!Rst_n) begin
        State       <= S_NORMAL;
        DirOut      <= `DIR_STOP;
        SpeedOut    <= 8'd0;
        AvoidActive <= 1'b0;
        Near        <= 1'b0;
        hold_cnt    <= 16'd0;
    end
    else if(!En) begin

        State       <= S_NORMAL;
        DirOut      <= CmdDir;
        SpeedOut    <= CmdSpeed;
        AvoidActive <= 1'b0;
        Near        <= 1'b0;
        hold_cnt    <= 16'd0;
    end
    else if(Tick) begin
        case(State)

        S_NORMAL: begin
            DirOut      <= CmdDir;
            SpeedOut    <= CmdSpeed;
            AvoidActive <= 1'b0;
            Near        <= 1'b0;
            hold_cnt    <= 16'd0;

            if(f_stop)      State <= S_STOP;
            else if(f_slow) State <= S_SLOW;
        end

        S_SLOW: begin
            DirOut      <= CmdDir;
            SpeedOut    <= slow_spd[7:0];
            AvoidActive <= 1'b1;
            Near        <= 1'b0;

            if(f_stop)      begin State <= S_STOP; hold_cnt <= 16'd0; end
            else if(f_clear) State <= S_NORMAL;
        end

        S_STOP: begin
            DirOut      <= `DIR_STOP;
            SpeedOut    <= 8'd0;
            AvoidActive <= 1'b1;
            Near        <= 1'b1;

            if(f_clear) begin
                State    <= S_RESUME;
                hold_cnt <= 16'd0;
            end
            else if(hold_cnt >= STUCK_CNT - 1) begin
                State    <= S_BACKUP;
                hold_cnt <= 16'd0;
            end
            else hold_cnt <= hold_cnt + 1'b1;
        end

        S_BACKUP: begin
            DirOut      <= `DIR_BACK;
            SpeedOut    <= BACK_SPEED;
            AvoidActive <= 1'b1;
            Near        <= 1'b1;

            if(r_block) begin                         
                State    <= S_TURN;
                hold_cnt <= 16'd0;
            end
            else if(hold_cnt >= BACK_CNT - 1) begin
                State    <= S_TURN;
                hold_cnt <= 16'd0;
            end
            else hold_cnt <= hold_cnt + 1'b1;
        end

        S_TURN: begin
            DirOut      <= turn_dir_w;
            SpeedOut    <= TURN_SPEED;
            AvoidActive <= 1'b1;
            Near        <= 1'b1;

            if(f_clear) begin                         
                State    <= S_RESUME;
                hold_cnt <= 16'd0;
            end
            else if(hold_cnt >= TURN_CNT - 1) begin   
                State    <= S_RESUME;
                hold_cnt <= 16'd0;
            end
            else hold_cnt <= hold_cnt + 1'b1;
        end

        S_RESUME: begin
            DirOut      <= `DIR_FWD;
            SpeedOut    <= CmdSpeed;
            AvoidActive <= 1'b1;
            Near        <= 1'b0;

            if(f_stop) begin
                State    <= S_STOP;
                hold_cnt <= 16'd0;
            end
            else if(hold_cnt >= RESUME_CNT - 1) begin
                State    <= S_NORMAL;
                hold_cnt <= 16'd0;
            end
            else hold_cnt <= hold_cnt + 1'b1;
        end

        default: State <= S_NORMAL;
        endcase
    end

endmodule
