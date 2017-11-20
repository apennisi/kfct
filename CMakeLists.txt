cmake_minimum_required(VERSION 2.8)
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3")
project( ct )

set(CMAKE_CXX_FLAGS "-std=c++11 -std=c++0x")

find_package(OpenCV REQUIRED)


set(EXECUTABLE_OUTPUT_PATH ../bin)

include_directories( src )

include_directories( ${OpenCV_INCLUDE_DIRS} )

file(GLOB_RECURSE ct_SRC "src/*.cpp")
add_executable( ct ${ct_SRC})
target_link_libraries( ct ${OpenCV_LIBS})
