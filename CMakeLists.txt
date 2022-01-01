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
	add_executable(bundle_adjuster_example1 src/bundle_adjuster_example1.cpp)
	target_link_libraries(bundle_adjuster_example1  ${OpenCV_LIBS}  ${CERES_LIBRARIES})

	add_executable(bundle_adjuster_example2 src/bundle_adjuster_example2.cpp src/bal_problem.cpp)
	target_link_libraries(bundle_adjuster_example2  ${OpenCV_LIBS}  ${CERES_LIBRARIES} gflags)
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

#if(EXISTS "$ENV{HOME}/usr/share/OpenCV/")
if(EXISTS "$ENV{HOME}/usr/share/opencv4/")

#	set(OpenCV_DIR "$ENV{HOME}/usr/lib/cmake/opencv4/")
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


#add_executable(tracking_objects_by_color src/tracking_objects_by_color.cpp)
#target_link_libraries(tracking_objects_by_color ${OpenCV_LIBS} )

add_executable(BasicOperations src/BasicOperations.cpp)
target_link_libraries(BasicOperations ${OpenCV_LIBS} )


#add_executable(OpenCV3BasicOperations src/OpenCV3BasicOperations.cpp)
#target_link_libraries(OpenCV3BasicOperations  ${OpenCV_LIBS} )


#add_executable(homography_example src/homography_example.cpp)
#target_link_libraries(homography_example ${OpenCV_LIBS} )


add_executable(affine_transformation_example src/affine_transformation_example.cpp)
target_link_libraries(affine_transformation_example ${OpenCV_LIBS} )

#add_executable(perspective-n-point src/perspective-n-point.cpp)
#target_link_libraries(perspective-n-point ${OpenCV_LIBS} )

#add_executable(camera_calibration_example src/camera_calibration_example.cpp)
#target_link_libraries(camera_calibration_example ${OpenCV_LIBS} )

#add_executable(stereo_rectify_uncalibrated src/stereo_rectify_uncalibrated.cpp)
#target_link_libraries(stereo_rectify_uncalibrated ${OpenCV_LIBS} )

#add_executable(stereo_calibration_example src/stereo_calibration_example.cpp)
#target_link_libraries(stereo_calibration_example ${OpenCV_LIBS} )

#add_executable(stereo_calib src/stereo_calib.cpp)
#target_link_libraries(stereo_calib ${OpenCV_LIBS} )

#add_executable(stereo_block_matching src/stereo_block_matching.cpp)
#target_link_libraries(stereo_block_matching ${OpenCV_LIBS} )


add_executable(fundamental_matrix_estimation src/fundamental_matrix_estimation.cpp)
target_link_libraries(fundamental_matrix_estimation ${OpenCV_LIBS} )


######################################### optical flow #########################################
#add_executable(optical_flow src/optical_flow.cpp)
#target_link_libraries(optical_flow ${OpenCV_LIBS} )

#add_executable(optical_Flow_demo src/optical_Flow_demo.cpp)
#target_link_libraries(optical_Flow_demo ${OpenCV_LIBS} )

#add_executable(pyrlk_optical_flow src/pyrlk_optical_flow.cpp)
#target_link_libraries(pyrlk_optical_flow ${OpenCV_LIBS} )

#add_executable(lkdemo src/lkdemo.cpp)
#target_link_libraries(lkdemo ${OpenCV_LIBS} )

#add_executable(fback src/fback.cpp)
#target_link_libraries(fback ${OpenCV_LIBS} )

#add_executable(flow_video src/flow_video.cpp)
#target_link_libraries(flow_video ${OpenCV_LIBS} )



#add_executable(visual_odometry src/visual_odometry.cpp)
#target_link_libraries(visual_odometry ${OpenCV_LIBS} )




#add_executable(sfm_example src/sfm_example.cpp)
#target_link_libraries(sfm_example ${OpenCV_LIBS} )

#add_executable(example_sfm_scene_reconstruction src/example_sfm_scene_reconstruction.cpp)
#target_link_libraries(example_sfm_scene_reconstruction ${OpenCV_LIBS} )

#add_executable(example_sfm_trajectory_reconstruction src/example_sfm_trajectory_reconstruction.cpp)
#target_link_libraries(example_sfm_trajectory_reconstruction ${OpenCV_LIBS} )



#add_executable(icp src/icp.cpp)
#target_link_libraries(icp  ${OpenCV_LIBS})





