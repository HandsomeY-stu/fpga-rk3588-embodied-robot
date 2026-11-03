/*
 * car_defines.vh
 * car_defines模块的实现或辅助入口，所属功能见代码索引。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
`ifndef CAR_DEFINES_VH
`define CAR_DEFINES_VH

`define MOTION_EN           1    
`define AVOID_EN            1    
`define NAV_EN              1    
`define VISION_EN           0    
`define TELEM_EN            1    
`define LEGACY_1BYTE_EN     0    

`define CLK_CTRL_HZ         50_000_000   
`define CLK_DISP_HZ         74_250_000   

`define CTRL_PERIOD_MS      10           
`define TELEM_PERIOD_MS     50           
`define STATUS_PERIOD_MS    200          

`define CTRL_PERIOD_CNT     (`CLK_CTRL_HZ/1000*`CTRL_PERIOD_MS)
`define TELEM_PERIOD_CNT    (`CLK_CTRL_HZ/1000*`TELEM_PERIOD_MS)
`define STATUS_PERIOD_CNT   (`CLK_CTRL_HZ/1000*`STATUS_PERIOD_MS)

`define UART_BAUD           115200       
`define IMU_BAUD            9600         

`define FRAME_HEAD_H        8'hA5
`define FRAME_HEAD_L        8'h5A
`define FRAME_LEN_MIN       8'd1         
`define FRAME_LEN_MAX       8'd32        
`define FRAME_OVERHEAD      8'd6         
`define FRAME_TIMEOUT_MS    20           

`define CRC8_POLY           8'h07
`define CRC8_INIT           8'h00

`define CMD_MOTION          8'h01     
`define CMD_SPEED           8'h02     
`define CMD_MODE            8'h03     
`define CMD_STOP            8'h04     
`define CMD_ROTATE          8'h05     

`define CMD_BEEP            8'h06     
`define CMD_WAYPOINT        8'h10     
`define CMD_NAV_START       8'h11     
`define CMD_NAV_ABORT       8'h12     
`define CMD_SEG1_START      8'h13     
`define CMD_SEG1_ABORT      8'h14     
`define CMD_VISION          8'h20     
`define CMD_VISION_EN       8'h21     
`define CMD_OSD_MODE        8'h22     
`define CMD_PID_SET         8'h30     
`define CMD_PING            8'h7E     
`define CMD_QUERY           8'h7F     
`define CMD_SOFT_RST        8'hF0     

`define QUERY_ALL           8'h00
`define QUERY_ATTITUDE      8'h01
`define QUERY_RANGE         8'h02
`define QUERY_ODOM          8'h03
`define QUERY_STATUS        8'h04

`define TLM_ATTITUDE        8'h81     
`define TLM_RANGE           8'h82     
`define TLM_ODOM            8'h83     
`define TLM_STATUS          8'h84     
`define TLM_NAV             8'h85     
`define TLM_VISION          8'h86     
`define TLM_PONG            8'h8F     
`define TLM_FAULT           8'h9F     

`define DIR_STOP            8'h00     
`define DIR_FWD             8'h01     
`define DIR_BACK            8'h02     
`define DIR_LEFT            8'h03     
`define DIR_RIGHT           8'h04     
`define DIR_SPIN_L          8'h05     
`define DIR_SPIN_R          8'h06     

`define MODE_MANUAL         8'h00     
`define MODE_AVOID          8'h01     
`define MODE_NAV            8'h02     
`define MODE_VISION         8'h03     

`define MODE_SEG1          8'h04     

`define NAV_IDLE            8'h00     
`define NAV_GO              8'h01     
`define NAV_TURN            8'h02     
`define NAV_AVOID           8'h03     
`define NAV_ARRIVED         8'h04     
`define NAV_DONE            8'h05     

`define FAULT_IMU_NODATA    0        
`define FAULT_US_TIMEOUT    1        
`define FAULT_ENC_NOPULSE   2        
`define FAULT_MOTOR_DRV     3        
`define FAULT_LINK_LOST     4        
`define FAULT_LOW_BAT       5        
`define FAULT_ESTOP         6        

`define PWM_FREQ_HZ         20000    
`define PWM_STEPS           1000     

`define PWM_PERIOD_CNT      (`CLK_CTRL_HZ/`PWM_FREQ_HZ - 1)
`define PWM_DEAD_TIME_NS    1000     

`define DUTY_MAX            1000
`define DUTY_MIN            (-1000)

`define MOTOR_L_INVERT      1        
`define MOTOR_R_INVERT      1        
`define ENC_L_INVERT        0        
`define ENC_R_INVERT        1        

`define US_NUM              3        
`define US_TRIG_PERIOD_MS   60       
`define US_TIMEOUT_MS       30       
`define US_MIN_MM           20       
`define US_MAX_MM           4000     
`define US_INVALID          16'hFFFF 

`define AVOID_SLOW_MM       600      
`define AVOID_STOP_MM       250      
`define AVOID_CLEAR_MM      800      
`define AVOID_SLOW_SCALE    50       

`define IMU_TIMEOUT_MS      200      
`define LINK_TIMEOUT_MS     1000     

`define ANG_RAW2D100(raw)   (((raw) * 32'd18000) >> 15)

`define ATT_ROLL_SIGN       +1       
`define ATT_PITCH_SIGN      -1       
`define ATT_YAW_SIGN        -1       

`define YAW_ZERO_CAL_MS     2000

`define US_US2MM_K          32'd2785
`define US_US2MM(us)        (((us) * `US_US2MM_K) >> 14)

`define WHEEL_DIAM_MM       65       
`define ENC_CPR             13       
`define GEAR_RATIO          30       
`define ENC_QUAD            4        

`define PIN_IMU_RX          "A13"    

`define PIN_MOTOR_PWM0      "A15"    
`define PIN_MOTOR_DIR0      "A16"    
`define PIN_MOTOR_PWM1      "A18"    
`define PIN_MOTOR_DIR1      "A19"    
`define PIN_MOTOR_PWM2      "F13"    
`define PIN_MOTOR_DIR2      "F14"    
`define PIN_MOTOR_PWM3      "E13"    
`define PIN_MOTOR_DIR3      "E14"    

`define PIN_US_TRIG0        "C13"    
`define PIN_US_ECHO0        "B13"    
`define PIN_US_TRIG1        "D14"    
`define PIN_US_ECHO1        "D15"    
`define PIN_US_TRIG2        "C14"    
`define PIN_US_ECHO2        "C15"    

`define PIN_ENC_A0          "B15"    
`define PIN_ENC_B0          "B16"    
`define PIN_ENC_A1          "D17"    
`define PIN_ENC_B1          "C17"    

`define PIN_UART1_TX        "E16"    
`define PIN_UART1_RX        "D16"    

`define PIN_UART0_RX        "J21"    
`define PIN_UART0_TX        "M15"    

`define PIN_KEY_S1          "A20"    
`define PIN_KEY_S2          "B20"    
`define PIN_KEY_S3          "A21"    
`define PIN_KEY_S4          "B21"    
`define PIN_DIP_SW0         "G22"    
`define PIN_DIP_SW7         "B22"    
`define PIN_BEEP            "N19"    

`define MUL_SHIFT(a, b, s)  (((a) * (b)) >> (s))    

`define ANG_RAW2D100_K      32'd18000
`define ANG_D100_DIV100_K   32'd41944
`define DIV10_K             32'd820

`endif   
