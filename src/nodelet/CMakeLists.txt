add_library(wit_nodelet wit_nodelet.cpp)
add_dependencies(wit_nodelet wit_ros)
target_link_libraries(wit_nodelet wit_ros)

