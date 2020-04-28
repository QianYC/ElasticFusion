cmake_minimum_required(VERSION 3.12)
project(dk_test)

set(CMAKE_CXX_STANDARD 14)

find_package(k4a REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        ${k4a_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(dk_test main.cpp)

target_link_libraries(dk_test ${OpenCV_LIBS} ${PROJECT_SOURCE_DIR}/lib/libk4a.so.1.1)