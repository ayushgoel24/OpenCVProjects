cmake_minimum_required(VERSION 3.1 )
project( OpenCVProjects )
set(CMAKE_BUILD_TYPE Debug)

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

MESSAGE("EIGEN3_VERSION: " ${EIGEN3_VERSION})
INCLUDE_DIRECTORIES(${EIGEN3_INCLUDE_DIR})
set(OpenCV_DIR "$ENV{HOME}/usr/lib/cmake/opencv4")


message("\n########################################## Ceres ##########################################\n")
ADD_DEFINITIONS(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})

if(EXISTS "$ENV{HOME}/usr/lib/cmake/Ceres")
	set(Ceres_DIR  "$ENV{HOME}/usr/lib/cmake/Ceres")	
endif()

set(glog_DIR "$ENV{HOME}/usr/lib/cmake/glog/")
FIND_PACKAGE(glog REQUIRED google-glog glog)
MESSAGE("GLOG_FOUND: " ${glog_FOUND})
MESSAGE("GLOG_INCLUDE_DIRS: " ${GLOG_INCLUDE_DIRS})
MESSAGE("GLOG_LIBRARIES: " ${GLOG_LIBRARY})




set(Ceres_DIR "$ENV{HOME}/usr/lib/cmake/Ceres")
FIND_PACKAGE(Ceres REQUIRED)
MESSAGE("CERES_FOUND: " ${CERES_FOUND})
MESSAGE("CERES_VERSION: " ${CERES_VERSION})
MESSAGE("CERES_LIBRARIES: " ${CERES_LIBRARIES})

find_package (Ceres QUIET)
if(${CERES_FOUND})
	message("Ceres_VERSION: " ${Ceres_VERSION})
	message("CERES_INCLUDE_DIRS: " ${CERES_INCLUDE_DIRS})
	message("CERES_LIBRARIES: " ${CERES_LIBRARIES})
	#add_executable(bundle_adjuster_example1 src/bundle_adjuster_example1.cpp)
	#target_link_libraries(bundle_adjuster_example1  ${OpenCV_LIBS}  ${CERES_LIBRARIES})

	#add_executable(bundle_adjuster_example2 src/bundle_adjuster_example2.cpp src/bal_problem.cpp)
	#target_link_libraries(bundle_adjuster_example2  ${OpenCV_LIBS}  ${CERES_LIBRARIES} gflags)
else()  	
	message("Ceres not found")
endif()	


message("\n########################################## Eigen ##########################################\n")





message("\n########################################## gflag ##########################################\n")

if(EXISTS "$ENV{HOME}/usr/lib/cmake/gflags/")
	set(gflags_DIR "$ENV{HOME}/usr/lib/cmake/gflags/")	
endif()
find_package(gflags QUIET)
if(${GFLAGS_FOUND})
	message("GFLAGS_FOUND: " ${GFLAGS_FOUND})
	message("GFLAGS_INCLUDE_DIRS: " ${GFLAGS_INCLUDE_DIRS})
	message("GFLAGS_LIBRARIES: " ${GFLAGS_LIBRARIES})
	message("GFLAGS_LIBRARYRARY_DIRS: " ${GFLAGS_LIBRARYRARY_DIRS} )
	add_definitions(-DCERES_GFLAGS_NAMESPACE=${GFLAGS_NAMESPACE})
else()
	message("gflags not found")
endif()



message("\n########################################## OpenCV ##########################################\n")


#ADD_EXECUTABLE(affine_transformation_example src/affine_transformation_example.cpp)
#TARGET_LINK_LIBRARIES(affine_transformation_example ${OpenCV_LIBS} )
endif()
find_package(OpenCV REQUIRED)
if(${OpenCV_FOUND})
	message("OpenCV_DIR: " ${OpenCV_DIR})
	message("OpenCV_VERSION: " ${OpenCV_VERSION})
	message("OpenCV_INCLUDE_DIRS: " ${OpenCV_INCLUDE_DIRS})
	message("OpenCV_LIBS: " ${OpenCV_LIBS} )
else()
	message("OpenCV not found")
endif()


add_executable(basic_operations src/basic_operations.cpp)
target_link_libraries(basic_operations ${OpenCV_LIBS} )

add_executable(hough_transform src/hough_transform.cpp)
target_link_libraries(hough_transform ${OpenCV_LIBS} )

add_executable(feature_detection src/feature_detection.cpp)
target_link_libraries(feature_detection ${OpenCV_LIBS} )

add_executable(edge_detection src/edge_detection.cpp)
target_link_libraries(edge_detection ${OpenCV_LIBS} )

add_executable(blob_detection src/blob_detection.cpp)
target_link_libraries(blob_detection ${OpenCV_LIBS} )

add_executable(feature_description src/feature_description.cpp)
target_link_libraries(feature_description ${OpenCV_LIBS} )
#TARGET_LINK_LIBRARIES(optical_flow ${OpenCV_LIBS} )

add_executable(image_moments src/image_moments.cpp)
target_link_libraries(image_moments ${OpenCV_LIBS} )

add_executable(histogram_analysis src/histogram_analysis.cpp)
target_link_libraries(histogram_analysis ${OpenCV_LIBS} )

add_executable(affine_transformation src/affine_transformation.cpp)
target_link_libraries(affine_transformation ${OpenCV_LIBS} )


add_executable(fundamental_matrix_estimation src/fundamental_matrix_estimation.cpp)
target_link_libraries(fundamental_matrix_estimation ${OpenCV_LIBS} )

#add_executable(homography src/homography.cpp)
#target_link_libraries(homography ${OpenCV_LIBS} )

add_executable(icp src/icp.cpp)
target_link_libraries(icp ${OpenCV_LIBS} )

add_executable(perspective-n-point src/perspective-n-point.cpp)
target_link_libraries(perspective-n-point ${OpenCV_LIBS} )

add_executable(homogenious_conversion src/homogenious_conversion.cpp)
target_link_libraries(homogenious_conversion ${OpenCV_LIBS} )

add_executable(pca src/pca.cpp)
target_link_libraries(pca ${OpenCV_LIBS} )

add_executable(morphological_transformation src/morphological_transformation.cpp)
target_link_libraries(morphological_transformation ${OpenCV_LIBS} )

add_executable(image_thresholding src/image_thresholding.cpp)
target_link_libraries(image_thresholding ${OpenCV_LIBS} )

#add_executable(camera_calibration_example src/camera_calibration_example.cpp)
#target_link_libraries(camera_calibration_example ${OpenCV_LIBS} )

add_executable(epipolar_geometry src/epipolar_geometry.cpp)
target_link_libraries(epipolar_geometry ${OpenCV_LIBS} )

add_executable(visual_odometry src/visual_odometry.cpp)
target_link_libraries(visual_odometry ${OpenCV_LIBS} )

add_executable(optical_flow src/optical_flow.cpp)
target_link_libraries(optical_flow ${OpenCV_LIBS} )

add_executable(tracking_objects_by_color src/tracking_objects_by_color.cpp)
target_link_libraries(tracking_objects_by_color ${OpenCV_LIBS} )

add_executable(essential_matrix_estimation src/essential_matrix_estimation.cpp)
target_link_libraries(essential_matrix_estimation ${OpenCV_LIBS} )

add_executable(image_morphe src/image_morphe.cpp)
target_link_libraries(image_morphe ${OpenCV_LIBS} )

add_executable(triangulation src/triangulation.cpp)
target_link_libraries(triangulation ${OpenCV_LIBS} )

add_executable(structured_light_range_finding src/structured_light_range_finding.cpp)
target_link_libraries(structured_light_range_finding ${OpenCV_LIBS} )

add_executable(2d_image_point_to_3d_world_unit_vector src/2d_image_point_to_3d_world_unit_vector.cpp)
target_link_libraries(2d_image_point_to_3d_world_unit_vector ${OpenCV_LIBS} )

#ADD_EXECUTABLE(fundamental_matrix_estimation src/fundamental_matrix_estimation.cpp)
#TARGET_LINK_LIBRARIES(fundamental_matrix_estimation ${OpenCV_LIBS} )

add_executable(convolution src/convolution.cpp)
target_link_libraries(convolution ${OpenCV_LIBS} )



if(EXISTS "${CMAKE_SOURCE_DIR}/src/sandbox.cpp")
    add_executable(sandbox src/sandbox.cpp)
    target_link_libraries(sandbox ${OpenCV_LIBS} )
endif()
TARGET_LINK_LIBRARIES(fundamental_essential_matrix_recover_pose  ${OpenCV_LIBS} -fopenmp)


ADD_EXECUTABLE(decompose_projection_matrix src/decompose_projection_matrix.cpp)
TARGET_LINK_LIBRARIES(decompose_projection_matrix  ${OpenCV_LIBS} -fopenmp)

ADD_EXECUTABLE(kalman_Example src/kalman_Example.cpp)
TARGET_LINK_LIBRARIES(kalman_Example  ${OpenCV_LIBS} -fopenmp)


ADD_EXECUTABLE(tracking_example src/tracking_example.cpp)
TARGET_LINK_LIBRARIES(tracking_example  ${OpenCV_LIBS} -fopenmp)

ADD_EXECUTABLE(projection_example src/projection_example.cpp)
TARGET_LINK_LIBRARIES(projection_example  ${OpenCV_LIBS} -fopenmp )

ADD_EXECUTABLE(rotation_matrices src/rotation_matrices.cpp)
TARGET_LINK_LIBRARIES(rotation_matrices  ${OpenCV_LIBS} -fopenmp)

ADD_EXECUTABLE(eigen_opencv_matrix_conversion src/eigen_opencv_matrix_conversion.cpp)
TARGET_LINK_LIBRARIES(eigen_opencv_matrix_conversion ${OpenCV_LIBS} -fopenmp)

ADD_EXECUTABLE(fundamental_matrix src/fundamental_matrix.cpp)
TARGET_LINK_LIBRARIES(fundamental_matrix ${OpenCV_LIBS} -fopenmp)

ADD_EXECUTABLE(essential_matrix src/essential_matrix.cpp)
TARGET_LINK_LIBRARIES(essential_matrix ${OpenCV_LIBS} -fopenmp)

ADD_EXECUTABLE(recover_pose src/recover_pose.cpp)
TARGET_LINK_LIBRARIES(recover_pose ${OpenCV_LIBS} -fopenmp)

ADD_EXECUTABLE(bundle_adjustment src/bundle_adjustment.cpp)
TARGET_LINK_LIBRARIES(bundle_adjustment ${OpenCV_LIBS} -fopenmp ${GFLAGS_LIBRARIES} ${CERES_LIBRARIES} ${GLOG_LIBRARY})

