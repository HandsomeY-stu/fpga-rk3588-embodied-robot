/*
 * osd_mode_cdc.v
 * OSD模式的跨时钟握手：稳定数据保持至应答，显示模式在帧边界提交。
 * 接口与参数保持交付版本；第三方版权与综合指令按原文件保留。
 */
module osd_mode_cdc(
    input SrcClk, input DstClk, input Rst_n,
    input [1:0] DesiredMode, input FrameBegin,
    output reg [1:0] DisplayMode
);
    reg [1:0] data_hold;
    reg req, ack;
    (* syn_preserve=1 *) reg ack_meta, ack_sync;
    (* syn_preserve=1 *) reg req_meta, req_sync;
    (* syn_preserve=1 *) reg [1:0] data_meta, data_sync;
    reg [1:0] pending_mode;
    // 发布后保持data_hold不变，直到应答经过两级同步；快速命令合并到最新期望模式。
    always @(posedge SrcClk or negedge Rst_n) begin
        if (!Rst_n) begin
            data_hold <= 2'd1; req <= 0; ack_meta <= 0; ack_sync <= 0;
        end else begin
            ack_meta <= ack; ack_sync <= ack_meta;
            if ((ack_sync == req) && (DesiredMode != data_hold)) begin
                data_hold <= DesiredMode; req <= ~req;
            end
        end
    end
    always @(posedge DstClk or negedge Rst_n) begin
        if (!Rst_n) begin
            req_meta <= 0; req_sync <= 0; ack <= 0;
            data_meta <= 2'd1; data_sync <= 2'd1;
            pending_mode <= 2'd1; DisplayMode <= 2'd1;
        end else begin
            req_meta <= req; req_sync <= req_meta;
            data_meta <= data_hold; data_sync <= data_meta;

            if (req_sync != ack) begin
                pending_mode <= data_sync;
                ack <= req_sync;
            end
            // 只在帧边界提交显示模式，避免半帧切换和跨域选择器直接驱动RGB数据。
            if (FrameBegin) DisplayMode <= pending_mode;
        end
    end
endmodule
