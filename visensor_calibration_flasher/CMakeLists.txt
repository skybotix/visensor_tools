cmake_minimum_required(VERSION 2.8.3)
project(visensor_calibration_flasher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(libvisensor 2.3.0 REQUIRED)
include_directories(include ${catkin_INCLUDE_DIRS} ${libvisensor_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x -D__STRICT_ANSI__")

catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
  roscpp 
  libvisensor
)

add_executable(visensor_calibration_flasher src/main.cpp src/visensor_calibration_flasher.cpp )

target_link_libraries(visensor_calibration_flasher ${libvisensor_LIBRARIES} ${catkin_LIBRARIES})
