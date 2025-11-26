cmake_minimum_required(VERSION 3.10)
project(can-socket C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(${PROJECT_NAME} STATIC
	${CMAKE_CURRENT_SOURCE_DIR}/src/CANSocket.cpp
)

target_include_directories( ${PROJECT_NAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/include
)

find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE 
	Threads::Threads)

add_executable(sendloop
	${CMAKE_CURRENT_SOURCE_DIR}/examples/sendloop.cpp 
)

target_link_libraries(sendloop PUBLIC
	${PROJECT_NAME})
