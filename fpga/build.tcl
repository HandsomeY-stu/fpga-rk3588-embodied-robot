set project_dir [file dirname [file normalize [info script]]]
open_project [file join $project_dir ov5640_ddr3_hdmi.gprj]
set_option -include_path [list [file join $project_dir src common]]
set_option -top_module gao_ov5640_ddr_hdmi
set_option -use_sspi_as_gpio 1
set_option -timing_driven 1
set_option -place_option 4
run syn
run pnr
