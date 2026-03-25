add_subdirectory(navigation)
add_subdirectory(temperature)
add_subdirectory(thrust)

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

target_link_libraries(modules__aviant PRIVATE navigation temperature thrust)
