file(GLOB SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)
add_library(wit_driver ${SOURCES})
target_link_libraries(wit_driver ${catkin_LIBRARIES})
