#Skybotix AG, 2/12/2013
cmake_minimum_required(VERSION 2.8.0)
 
############
# SETTINGS
############
set(PKGNAME visensor_update)

SET(EXECUTABLE_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib)

###########
# BUILD
###########
FILE(GLOB SRCS src/*.cpp src/communication_layers/*.cpp)

#Release / debug build
set(CMAKE_BUILD_TYPE Release)
ADD_DEFINITIONS (-march=native -O2 -fmessage-length=0 -MMD -MP -Wall -pedantic -std=c++0x ) #Release
#ADD_DEFINITIONS (-march=native -std=c++0x -O0 -g3 -fmessage-length=0 -MMD -MP -Wall -pedantic -std=c++0x ) #Debug

#executable
ADD_EXECUTABLE(${PKGNAME} ${SRCS})
TARGET_LINK_LIBRARIES(${PKGNAME} boost_regex curl ssh2) 

###########
# INSTALL
###########
#visensor_update
INSTALL(
  FILES bin/visensor_update
  DESTINATION bin 
  COMPONENT tools
)

