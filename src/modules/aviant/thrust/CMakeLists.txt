px4_add_library(thrust
	Thrust.cpp
)
target_link_libraries(thrust
	PRIVATE
		px4_work_queue
)
