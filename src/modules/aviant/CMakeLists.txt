add_subdirectory(mr_thrust_indicator)
add_subdirectory(navigation)

px4_add_module(
	MODULE modules__aviant
	MAIN aviant
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}
	COMPILE_FLAGS
	SRCS
		aviant.cpp
	MODULE_CONFIG
		module.yaml
)

target_link_libraries(modules__aviant PRIVATE mr_thrust_indicator)
target_link_libraries(modules__aviant PRIVATE navigation)
