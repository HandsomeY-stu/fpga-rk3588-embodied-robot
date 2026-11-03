# 2026-10-09: 720p20, byte PCLK nominal 56 MHz (DVP application guide).
# 0x4740[5]=1 launches on FALLING edge; capture on rising edge.
# 48 MHz in datasheet table 8-5 is TYPICAL, not a hard minimum.
# External tCO/board skew bounds are provisional (0..3 ns).
# These need board measurement/diagnostic confirmation; not formal IO signoff.
create_clock -name camera_pclk -period 17.857 [get_ports {camera_pclk}]
create_clock -name clk50M -period 20.000 [get_ports {clk50M}]
set_input_delay -clock camera_pclk -clock_fall -max 3.000 [get_ports {camera_data[*] camera_href camera_vsync}]
set_input_delay -clock camera_pclk -clock_fall -min 0.000 [get_ports {camera_data[*] camera_href camera_vsync}]
set_clock_uncertainty 0.500 -setup -to [get_clocks {camera_pclk clk50M}]
set_clock_uncertainty 0.200 -hold -to [get_clocks {camera_pclk clk50M}]
# PLL/CLKDIV and DDR IP clocks are derived by the tool. Inspect detailed paths;
# do not suppress all inter-clock paths with a global exception.
create_generated_clock -name clk_1k -source [get_ports {clk50M}] -master_clock clk50M -divide_by 50000 -add [get_nets {u_hex8/clk_1K}]
# 2026-11-02: only the first stage of the new one-bit ACK synchronizer.
# Mailbox source holds bundled data until ACK traverses ack_meta -> ack_sync.
# Do not cut ack_meta->ack_sync or any display-data path.
set_false_path -from [get_cells {u_osd_mode_cdc/ack_s0}] -to [get_cells {u_osd_mode_cdc/ack_meta_s0}]
