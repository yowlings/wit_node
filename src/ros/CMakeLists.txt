add_library(wit_ros wit_ros.cpp)
target_link_libraries(wit_ros wit_driver ${catkin_LIBRARIES} )
