cmake_minimum_required(VERSION 2.8)
PROJECT( opencvdemo )
set(CMAKE_BUILD_TYPE Debug)


FIND_PACKAGE( OpenCV REQUIRED )

MESSAGE("OpenCV_DIR: " ${OpenCV_DIR})
MESSAGE("OpenCV_VERSION: " ${OpenCV_VERSION})
MESSAGE("OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS})
MESSAGE("OpenCV_LIBS: " ${OpenCV_LIBS})



FIND_PACKAGE(Boost COMPONENTS system filesystem program_options REQUIRED)
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
LINK_DIRECTORIES(${Boost_LIBRARY_DIR})

#This will print all unsed variable, etc
ADD_DEFINITIONS (-g -Wall -fopenmp)



# Check for c++11 support
INCLUDE(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
IF(COMPILER_SUPPORTS_CXX11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ELSEIF(COMPILER_SUPPORTS_CXX0X)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
  MESSAGE(ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
ENDIF()


ADD_EXECUTABLE(tracking_objects_by_color src/tracking_objects_by_color.cpp)
TARGET_LINK_LIBRARIES(tracking_objects_by_color ${OpenCV_LIBS} )
