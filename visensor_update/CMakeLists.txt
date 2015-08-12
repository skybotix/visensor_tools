#Skybotix AG, 2/12/2013
cmake_minimum_required(VERSION 2.8.3)
 
############
# CATKIN STUFF
############
project(visensor_update)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(libvisensor 2.3.0 REQUIRED)


############
# SETTINGS
############
SET(PKGNAME visensor_update)
include_directories(include ${catkin_INCLUDE_DIRS} ${libvisensor_INCLUDE_DIRS})

#Release / debug build
#set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=native -Wall -std=c++0x -D__STRICT_ANSI__")
#ADD_DEFINITIONS (-march=native -O2 -fmessage-length=0 -MMD -MP -Wno-reorder -Wall -pedantic -std=c++0x ) #Release
# ADD_DEFINITIONS (-march=native -std=c++0x -O0 -g3 -fmessage-length=0 -MMD -MP -Wall  -Wno-reorder -pedantic -std=c++0x ) #Debug

catkin_package(
  INCLUDE_DIRS ${catkin_INCLUDE_DIRS}
  CATKIN_DEPENDS
  roscpp 
)

###########
# BUILD
###########
FILE(GLOB SRCS src/*.cpp src/communication_layers/*.cpp)

add_executable(visensor_update  ${SRCS})

TARGET_LINK_LIBRARIES(visensor_update  ${libvisensor_LIBRARIES} ${catkin_LIBRARIES} boost_regex curl)

