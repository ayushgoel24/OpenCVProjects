cmake_minimum_required(VERSION 3.1 )
project( OpenCVProjects )
set(CMAKE_BUILD_TYPE Debug)



message("\n########################################## Ceres ##########################################\n")

if(EXISTS "$ENV{HOME}/usr/lib/cmake/Ceres")
	set(Ceres_DIR  "$ENV{HOME}/usr/lib/cmake/Ceres")	
endif()


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


find_package (Eigen3 QUIET)
if(${EIGEN3_FOUND})
	include_directories(${EIGEN3_INCLUDE_DIR})
	message("EIGEN3_VERSION: " ${EIGEN3_VERSION})
	message("EIGEN3_INCLUDE_DIR: " ${EIGEN3_INCLUDE_DIR})
else()  	
	message("Eigen not found")
endif()



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


if(EXISTS "$ENV{HOME}/usr/share/opencv4/")
	set(OpenCV_DIR "$ENV{HOME}/usr/lib/cmake/opencv4/")	
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


if(EXISTS "${CMAKE_SOURCE_DIR}/src/sandbox.cpp")
    add_executable(sandbox src/sandbox.cpp)
    target_link_libraries(sandbox ${OpenCV_LIBS} )
endif()

