cmake_minimum_required(VERSION 2.8)
PROJECT( opencvdemo )
set(CMAKE_BUILD_TYPE Debug)


set(Ceres_DIR  "$ENV{HOME}/usr/lib/cmake/Ceres")
find_package (Ceres REQUIRED)
MESSAGE("Ceres_VERSION: " ${Ceres_VERSION})
MESSAGE("CERES_INCLUDE_DIRS: " ${CERES_INCLUDE_DIRS})
MESSAGE("CERES_LIBRARIES: " ${CERES_LIBRARIES})


#FIND_PACKAGE(Eigen REQUIRED)
find_package (Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})


#If opencv is installed in custom directory
#set(OpenCV_DIR "$ENV{HOME}/usr/share/OpenCV/")

FIND_PACKAGE( OpenCV REQUIRED )

MESSAGE("OpenCV_DIR: " ${OpenCV_DIR})
MESSAGE("OpenCV_VERSION: " ${OpenCV_VERSION})
MESSAGE("OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS})
MESSAGE("OpenCV_LIBS: " ${OpenCV_LIBS} )

set(gflags_DIR "$ENV{HOME}/usr/lib/cmake/gflags/")

find_package(gflags REQUIRED)
#find_package(gflags COMPONENTS nothreads_static)
MESSAGE("GFLAGS_FOUND: " ${GFLAGS_FOUND})
MESSAGE("GFLAGS_INCLUDE_DIRS: " ${GFLAGS_INCLUDE_DIRS})
MESSAGE("GFLAGS_LIBRARIES: " ${GFLAGS_LIBRARIES})
MESSAGE("GFLAGS_LIBRARYRARY_DIRS: " ${GFLAGS_LIBRARYRARY_DIRS} )
add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})




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


#ADD_EXECUTABLE(tracking_objects_by_color src/tracking_objects_by_color.cpp)
#TARGET_LINK_LIBRARIES(tracking_objects_by_color ${OpenCV_LIBS} )

#ADD_EXECUTABLE(BasicOperations src/BasicOperations.cpp)
#TARGET_LINK_LIBRARIES(BasicOperations ${OpenCV_LIBS} )

#ADD_EXECUTABLE(homography_example src/homography_example.cpp)
#TARGET_LINK_LIBRARIES(homography_example ${OpenCV_LIBS} )


#ADD_EXECUTABLE(stereo_calibration_example src/stereo_calibration_example.cpp)
#TARGET_LINK_LIBRARIES(stereo_calibration_example ${OpenCV_LIBS} )

#ADD_EXECUTABLE(stereo_block_matching src/stereo_block_matching.cpp)
#TARGET_LINK_LIBRARIES(stereo_block_matching ${OpenCV_LIBS} )


ADD_EXECUTABLE(affine_transformation_example src/affine_transformation_example.cpp)
TARGET_LINK_LIBRARIES(affine_transformation_example ${OpenCV_LIBS} )

ADD_EXECUTABLE(perspective-n-point src/perspective-n-point.cpp)
TARGET_LINK_LIBRARIES(perspective-n-point ${OpenCV_LIBS} )

#ADD_EXECUTABLE(camera_calibration_example src/camera_calibration_example.cpp)
#TARGET_LINK_LIBRARIES(camera_calibration_example ${OpenCV_LIBS} )

#ADD_EXECUTABLE(stereo_rectify_uncalibrated src/stereo_rectify_uncalibrated.cpp)
#TARGET_LINK_LIBRARIES(stereo_rectify_uncalibrated ${OpenCV_LIBS} )


ADD_EXECUTABLE(fundamental_matrix_estimation src/fundamental_matrix_estimation.cpp)
TARGET_LINK_LIBRARIES(fundamental_matrix_estimation ${OpenCV_LIBS} )


ADD_EXECUTABLE(optical_flow src/optical_flow.cpp)
TARGET_LINK_LIBRARIES(optical_flow ${OpenCV_LIBS} )



#ADD_EXECUTABLE(visual_odometry src/visual_odometry.cpp)
#TARGET_LINK_LIBRARIES(visual_odometry ${OpenCV_LIBS} )

#ADD_EXECUTABLE(lkdemo src/lkdemo.cpp)
#TARGET_LINK_LIBRARIES(lkdemo ${OpenCV_LIBS} )

#ADD_EXECUTABLE(pyrlk_optical_flow src/pyrlk_optical_flow.cpp)
#TARGET_LINK_LIBRARIES(pyrlk_optical_flow ${OpenCV_LIBS} )


#ADD_EXECUTABLE(fback src/fback.cpp)
#TARGET_LINK_LIBRARIES(fback ${OpenCV_LIBS} )


#ADD_EXECUTABLE(flow_video src/flow_video.cpp)
#TARGET_LINK_LIBRARIES(flow_video ${OpenCV_LIBS} )

#ADD_EXECUTABLE(optical_Flow_demo src/optical_Flow_demo.cpp)
#TARGET_LINK_LIBRARIES(optical_Flow_demo ${OpenCV_LIBS} )

#ADD_EXECUTABLE(sfm_example src/sfm_example.cpp)
#TARGET_LINK_LIBRARIES(sfm_example ${OpenCV_LIBS} )

#ADD_EXECUTABLE(example_sfm_scene_reconstruction src/example_sfm_scene_reconstruction.cpp)
#TARGET_LINK_LIBRARIES(example_sfm_scene_reconstruction ${OpenCV_LIBS} )

#ADD_EXECUTABLE(example_sfm_trajectory_reconstruction src/example_sfm_trajectory_reconstruction.cpp)
#TARGET_LINK_LIBRARIES(example_sfm_trajectory_reconstruction ${OpenCV_LIBS} )



#ADD_EXECUTABLE(stereo_calib src/stereo_calib.cpp)
#TARGET_LINK_LIBRARIES(stereo_calib ${OpenCV_LIBS} )


#ADD_EXECUTABLE(OpenCV3BasicOperations src/OpenCV3BasicOperations.cpp)
#TARGET_LINK_LIBRARIES(OpenCV3BasicOperations  ${OpenCV_LIBS} )

#ADD_EXECUTABLE(bundle_adjuster_example1 src/bundle_adjuster_example1.cpp)
#TARGET_LINK_LIBRARIES(bundle_adjuster_example1  ${OpenCV_LIBS}  ${CERES_LIBRARIES})

#ADD_EXECUTABLE(bundle_adjuster_example2 src/bundle_adjuster_example2.cpp src/bal_problem.cpp)
#TARGET_LINK_LIBRARIES(bundle_adjuster_example2  ${OpenCV_LIBS}  ${CERES_LIBRARIES} gflags)


ADD_EXECUTABLE(icp src/icp.cpp)
TARGET_LINK_LIBRARIES(icp  ${OpenCV_LIBS})





