/*
 * disp_parameter_cfg.v
 * 显示分辨率和消隐时序的参数定义。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
/*============================================================================
*
*  LOGIC CORE:          显示设备驱动硬件配置头文件	
*  MODULE NAME:         disp_parameter_cfg()
*  COMPANY:             武汉芯路恒科技有限公司
*                       http://xiaomeige.taobao.com
*  author:              小梅哥
*  Website:             www.corecourse.cn
*  REVISION HISTORY:  
*
*  Revision 1.0  04/10/2019
*  Description: Initial Release.
*
*  FUNCTIONAL DESCRIPTION:
===========================================================================*/

`define HW_VGA

`define MODE_RGB888

`ifdef HW_TFT43	 
	`define Resolution_480x272 1	 

`elsif HW_TFT50	 
	`define Resolution_800x480 1	 

`elsif HW_VGA		 

	`define Resolution_1280x720 1	 

`endif

`ifdef MODE_RGB888
	`define Red_Bits 8
	`define Green_Bits 8
	`define Blue_Bits 8

`elsif MODE_RGB565
	`define Red_Bits 5
	`define Green_Bits 6
	`define Blue_Bits 5
`endif

`ifdef Resolution_480x272
	`define H_Total_Time  12'd525
	`define H_Right_Border  12'd0
	`define H_Front_Porch  12'd2
	`define H_Sync_Time  12'd41
	`define H_Back_Porch  12'd2
	`define H_Left_Border  12'd0

	`define V_Total_Time  12'd286
	`define V_Bottom_Border  12'd0
	`define V_Front_Porch  12'd2
	`define V_Sync_Time  12'd10
	`define V_Back_Porch  12'd2
	`define V_Top_Border  12'd0

`elsif Resolution_640x480
	`define H_Total_Time  12'd800
	`define H_Right_Border  12'd8
	`define H_Front_Porch  12'd8
	`define H_Sync_Time  12'd96
	`define H_Back_Porch  12'd40
	`define H_Left_Border  12'd8

	`define V_Total_Time  12'd525
	`define V_Bottom_Border  12'd8
	`define V_Front_Porch  12'd2
	`define V_Sync_Time  12'd2
	`define V_Back_Porch  12'd25
	`define V_Top_Border  12'd8

`elsif Resolution_800x480
	`define H_Total_Time 12'd1056
	`define H_Right_Border 12'd0
	`define H_Front_Porch 12'd40
	`define H_Sync_Time 12'd128
	`define H_Back_Porch 12'd88
	`define H_Left_Border 12'd0

	`define V_Total_Time 12'd525
	`define V_Bottom_Border 12'd8
	`define V_Front_Porch 12'd2
	`define V_Sync_Time 12'd2
	`define V_Back_Porch 12'd25
	`define V_Top_Border 12'd8

`elsif Resolution_800x600
	`define H_Total_Time 12'd1056
	`define H_Right_Border 12'd0
	`define H_Front_Porch 12'd40
	`define H_Sync_Time 12'd128
	`define H_Back_Porch 12'd88
	`define H_Left_Border 12'd0

	`define V_Total_Time 12'd628
	`define V_Bottom_Border 12'd0
	`define V_Front_Porch 12'd1
	`define V_Sync_Time 12'd4
	`define V_Back_Porch 12'd23
	`define V_Top_Border 12'd0

`elsif Resolution_1024x600
	`define H_Total_Time 12'd1344
	`define H_Right_Border 12'd0
	`define H_Front_Porch 12'd24
	`define H_Sync_Time 12'd136
	`define H_Back_Porch 12'd160
	`define H_Left_Border 12'd0

	`define V_Total_Time 12'd628
	`define V_Bottom_Border 12'd0
	`define V_Front_Porch 12'd1
	`define V_Sync_Time 12'd4
	`define V_Back_Porch 12'd23
	`define V_Top_Border 12'd0

`elsif Resolution_1024x768
	`define H_Total_Time 12'd1344
	`define H_Right_Border 12'd0
	`define H_Front_Porch 12'd24
	`define H_Sync_Time 12'd136
	`define H_Back_Porch 12'd160
	`define H_Left_Border 12'd0

	`define V_Total_Time 12'd806
	`define V_Bottom_Border 12'd0
	`define V_Front_Porch 12'd3
	`define V_Sync_Time 12'd6
	`define V_Back_Porch 12'd29
	`define V_Top_Border 12'd0

`elsif Resolution_1280x720
	`define H_Total_Time 12'd1650
	`define H_Right_Border 12'd0
	`define H_Front_Porch 12'd110
	`define H_Sync_Time 12'd40
	`define H_Back_Porch 12'd220
	`define H_Left_Border 12'd0

	`define V_Total_Time 12'd750
	`define V_Bottom_Border 12'd0
	`define V_Front_Porch 12'd5
	`define V_Sync_Time 12'd5
	`define V_Back_Porch 12'd20
	`define V_Top_Border 12'd0

`elsif Resolution_1920x1080
	`define H_Total_Time 12'd2200
	`define H_Right_Border 12'd0
	`define H_Front_Porch 12'd88
	`define H_Sync_Time 12'd44
	`define H_Back_Porch 12'd148
	`define H_Left_Border 12'd0

	`define V_Total_Time 12'd1125
	`define V_Bottom_Border 12'd0
	`define V_Front_Porch 12'd4
	`define V_Sync_Time 12'd5
	`define V_Back_Porch 12'd36
	`define V_Top_Border 12'd0	

`endif

