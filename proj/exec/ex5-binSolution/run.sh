#!/bin/sh
export LD_LIBRARY_PATH=${PWD}/bin/linux:$LD_LIBRARY_PATH
bin/linux/cgv_viewer plugin:cg_fltk "type(shader_config):shader_path='${PWD}/glsl'" plugin:cg_icons plugin:cg_ext plugin:cmi_io  plugin:crg_stereo_view plugin:CG2_exercise45 "config:'${PWD}/config.def'"
