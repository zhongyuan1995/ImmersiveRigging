
cgv_find_package(cg_ext)

set(PLUGINS
	cg_fltk
	cg_icons
	cg_ext
	crg_stereo_view
)

# Source files
file(GLOB_RECURSE SOURCES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.cxx")
# Header files
file(GLOB_RECURSE HEADERS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.h")
# Shader files
file(GLOB_RECURSE SHADER_SOURCES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" "*.gl*")

include_directories(
	${cgv_INCLUDE_DIRS} 
	${cgv_INCLUDE_DIRS}/libs 
	${cgv_gl_INCLUDE_DIRS}
)

cgv_add_module(${COURSE_NAME}_exercise45 ${SOURCES} ${IMG_SOURCES} ${HEADERS} ${SHADER_SOURCES} ${IMAGES})
target_link_libraries(${COURSE_NAME}_exercise45 ${OPENGL_LIBRARIES} ${cgv_LIBRARIES} ANNf)
cgv_test_shaders(${COURSE_NAME}_exercise45 ${SHADER_SOURCES})
set_target_properties(${COURSE_NAME}_exercise45 PROPERTIES FOLDER "${FOLDER_NAME_APPLICATION_PLUGINS}")

set_target_properties(${COURSE_NAME}_exercise45 PROPERTIES VS_DEBUGGER_COMMAND_ARGUMENTS "plugin:cg_fltk type(shader_config):shader_path='${CGV_DIR}/libs/cgv_gl/glsl;${CMAKE_CURRENT_SOURCE_DIR}' plugin:cg_icons plugin:cg_ext plugin:cmi_io plugin:crg_stereo_view plugin:${COURSE_NAME}_exercise45 config:${CMAKE_CURRENT_SOURCE_DIR}/config.def")
set_target_properties(${COURSE_NAME}_exercise45 PROPERTIES VS_DEBUGGER_COMMAND $<IF:$<CONFIG:Debug>,${VIEWER_DEBUG_EXE},${VIEWER_EXE}>)
set_target_properties(${COURSE_NAME}_exercise45 PROPERTIES VS_DEBUGGER_WORKING_DIRECTORY ${CMAKE_SOURCE_DIR})

cgv_write_find_file(${COURSE_NAME}_exercise45)

