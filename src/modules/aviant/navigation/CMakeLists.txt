px4_add_library(navigation
	Navigation.cpp
)
target_link_libraries(navigation
	PRIVATE
		px4_work_queue
)
