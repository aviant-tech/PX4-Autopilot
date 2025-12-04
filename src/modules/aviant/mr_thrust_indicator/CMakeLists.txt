px4_add_library(mr_thrust_indicator
	MrThrustIndicator.cpp
)
target_link_libraries(mr_thrust_indicator
	PRIVATE
		px4_work_queue
)
