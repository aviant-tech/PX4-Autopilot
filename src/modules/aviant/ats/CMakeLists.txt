px4_add_module(
	MODULE modules__aviant_ats
	MAIN aviant_ats
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}
	COMPILE_FLAGS
	SRCS
		ATS.cpp
	DEPENDS
		px4_work_queue
	)