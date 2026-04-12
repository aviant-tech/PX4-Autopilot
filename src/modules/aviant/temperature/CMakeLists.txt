px4_add_library(temperature
	Temperature.cpp
)
target_link_libraries(temperature
	PRIVATE
		px4_work_queue
)
