// #include <opencv2/core.hpp>
// #include <opencv2/sfm.hpp>
// #include <opencv2/viz.hpp>
// #include <iostream>
// #include <fstream>
// #include <string>
// using namespace std;
// using namespace cv;
// using namespace cv::sfm;
// 
// bool camera_pov = false;
// void keyboard_callback(const viz::KeyboardEvent &event, void* cookie)
// {
//   if ( event.action == 0 &&!event.symbol.compare("s") )
//     camera_pov = !camera_pov;
// }
// /* Sample main code
//  */
// int main(int argc, char** argv)
// {
//  // Read input parameters
//  if ( argc != 5 )
//  {
// 
//    exit(0);
//  }
//  // Read images from text file
// 
// 
//  // Set the camera calibration matrix
//  const double f  = atof(argv[2]),
//               cx = atof(argv[3]), cy = atof(argv[4]);
//  Matx33d K = Matx33d( f, 0, cx,
//                       0, f, cy,
//                       0, 0,  1);
//  bool is_projective = true;
//  vector<Mat> Rs_est, ts_est, points3d_estimated;
//  reconstruct(points2d, Rs_est, ts_est, K, points3d_estimated, is_projective);
//  // Print output
//  cout << "\n----------------------------\n" << endl;
//  cout << "Reconstruction: " << endl;
//  cout << "============================" << endl;
//  cout << "Estimated 3D points: " << points3d_estimated.size() << endl;
//  cout << "Estimated cameras: " << Rs_est.size() << endl;
//  cout << "Refined intrinsics: " << endl << K << endl << endl;
//  cout << "3D Visualization: " << endl;
//  cout << "============================" << endl;
//  viz::Viz3d window_est("Estimation Coordinate Frame");
//             window_est.setBackgroundColor(); // black by default
//             window_est.registerKeyboardCallback(&keyboard_callback);
//  // Create the pointcloud
//  cout << "Recovering points  ... ";
//  // recover estimated points3d
//  vector<Vec3f> point_cloud_est;
//  for (int i = 0; i < points3d_estimated.size(); ++i)
//    point_cloud_est.push_back(Vec3f(points3d_estimated[i]));
//  cout << "[DONE]" << endl;
//  cout << "Recovering cameras ... ";
//  vector<Affine3d> path_est;
//  for (size_t i = 0; i < Rs_est.size(); ++i)
//    path_est.push_back(Affine3d(Rs_est[i],ts_est[i]));
//  cout << "[DONE]" << endl;
//  cout << "Rendering Trajectory  ... ";
//  cout << endl << "Press:                       " << endl;
//  cout <<         " 's' to switch the camera pov" << endl;
//  cout <<         " 'q' to close the windows    " << endl;
//  if ( path_est.size() > 0 )
//  {
//    // animated trajectory
//    int idx = 0, forw = -1, n = static_cast<int>(path_est.size());
//    while(!window_est.wasStopped())
//    {
//      for (size_t i = 0; i < point_cloud_est.size(); ++i)
//      {
//        Vec3d point = point_cloud_est[i];
//        Affine3d point_pose(Mat::eye(3,3,CV_64F), point);
//        char buffer[50];
//        sprintf (buffer, "%d", static_cast<int>(i));
//        viz::WCube cube_widget(Point3f(0.1,0.1,0.0), Point3f(0.0,0.0,-0.1), true, viz::Color::blue());
//                   cube_widget.setRenderingProperty(viz::LINE_WIDTH, 2.0);
//        window_est.showWidget("Cube"+string(buffer), cube_widget, point_pose);
//      }
//      Affine3d cam_pose = path_est[idx];
//      viz::WCameraPosition cpw(0.25); // Coordinate axes
//      viz::WCameraPosition cpw_frustum(K, 0.3, viz::Color::yellow()); // Camera frustum
//      if ( camera_pov )
//        window_est.setViewerPose(cam_pose);
//      else
//      {
//        // render complete trajectory
//        window_est.showWidget("cameras_frames_and_lines_est", viz::WTrajectory(path_est, viz::WTrajectory::PATH, 1.0, viz::Color::green()));
//        window_est.showWidget("CPW", cpw, cam_pose);
//        window_est.showWidget("CPW_FRUSTUM", cpw_frustum, cam_pose);
//      }
//      // update trajectory index (spring effect)
//      forw *= (idx==n || idx==0) ? -1: 1; idx += forw;
//      // frame rate 1s
//      window_est.spinOnce(1, true);
//      window_est.removeAllWidgets();
//    }
//  }
//   return 0;
// }


//#include <opencv2/sfm.hpp>
//#include <opencv2/viz.hpp>
//#include <opencv2/calib3d.hpp>
//#include <opencv2/core.hpp>

#define CERES_FOUND true

#include </home/behnam/usr/include/opencv2/sfm.hpp>
#include </home/behnam/usr/include/opencv2/viz.hpp>
#include </home/behnam/usr/include/opencv2/calib3d.hpp>
#include </home/behnam/usr/include/opencv2/core.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <fstream>
using namespace std;
using namespace cv;
using namespace cv::sfm;
static void help() {
  cout
      << "\n------------------------------------------------------------------------------------\n"
      << " This program shows the multiview reconstruction capabilities in the \n"
      << " OpenCV Structure From Motion (SFM) module.\n"
      << " It reconstruct a scene from a set of 2D images \n"
      << " Usage:\n"
      << "        example_sfm_scene_reconstruction <path_to_file> <f> <cx> <cy>\n"
      << " where: path_to_file is the file absolute path into your system which contains\n"
      << "        the list of images to use for reconstruction. \n"
      << "        f  is the focal length in pixels. \n"
      << "        cx is the image principal point x coordinates in pixels. \n"
      << "        cy is the image principal point y coordinates in pixels. \n"
      << "------------------------------------------------------------------------------------\n\n"
      << endl;
}
int getdir(const string _filename, vector<String> &files)
{
  ifstream myfile(_filename.c_str());
  if (!myfile.is_open()) {
    cout << "Unable to read file: " << _filename << endl;
    exit(0);
  } else {;
    size_t found = _filename.find_last_of("/\\");
    string line_str, path_to_file = _filename.substr(0, found);
    while ( getline(myfile, line_str) )
      files.push_back(path_to_file+string("/")+line_str);
  }
  return 1;
}
int main(int argc, char* argv[])
{
  // Read input parameters
  if ( argc != 5 )
  {
    help();
    exit(0);
  }
  // Parse the image paths
  vector<String> images_paths;
  getdir( argv[1], images_paths );
  // Build intrinsics
  float f  = atof(argv[2]),
        cx = atof(argv[3]), cy = atof(argv[4]);
  Matx33d K = Matx33d( f, 0, cx,
                       0, f, cy,
                       0, 0,  1);
  bool is_projective = true;
  vector<Mat> Rs_est, ts_est, points3d_estimated;
  reconstruct(images_paths, Rs_est, ts_est, K, points3d_estimated, is_projective);
  // Print output
  cout << "\n----------------------------\n" << endl;
  cout << "Reconstruction: " << endl;
  cout << "============================" << endl;
  cout << "Estimated 3D points: " << points3d_estimated.size() << endl;
  cout << "Estimated cameras: " << Rs_est.size() << endl;
  cout << "Refined intrinsics: " << endl << K << endl << endl;
  cout << "3D Visualization: " << endl;
  cout << "============================" << endl;
  viz::Viz3d window("Coordinate Frame");
             window.setWindowSize(Size(500,500));
             window.setWindowPosition(Point(150,150));
             window.setBackgroundColor(); // black by default
  // Create the pointcloud
  cout << "Recovering points  ... ";
  // recover estimated points3d
  vector<Vec3f> point_cloud_est;
  for (int i = 0; i < points3d_estimated.size(); ++i)
  {
	  std::cout<< points3d_estimated[i].at<double>(0,0)<<" ";
	  std::cout<< points3d_estimated[i].at<double>(1,0)<< " ";
	  std::cout<< points3d_estimated[i].at<double>(2,0)<<std::endl;

	  point_cloud_est.push_back(Vec3f(points3d_estimated[i]));
	  
}
    
    
  cout << "[DONE]" << endl;
  cout << "Recovering cameras ... ";
  vector<Affine3d> path;
  for (size_t i = 0; i < Rs_est.size(); ++i)
    path.push_back(Affine3d(Rs_est[i],ts_est[i]));
  cout << "[DONE]" << endl;
  if ( point_cloud_est.size() > 0 )
  {
    cout << "Rendering points   ... ";
    viz::WCloud cloud_widget(point_cloud_est, viz::Color::green());
    window.showWidget("point_cloud", cloud_widget);
    cout << "[DONE]" << endl;
  }
  else
  {
    cout << "Cannot render points: Empty pointcloud" << endl;
  }
  if ( path.size() > 0 )
  {
    cout << "Rendering Cameras  ... ";
    window.showWidget("cameras_frames_and_lines", viz::WTrajectory(path, viz::WTrajectory::BOTH, 0.1, viz::Color::green()));
    window.showWidget("cameras_frustums", viz::WTrajectoryFrustums(path, K, 0.1, viz::Color::yellow()));
    window.setViewerPose(path[0]);
    cout << "[DONE]" << endl;
  }
  else
  {
    cout << "Cannot render the cameras: Empty path" << endl;
  }
  cout << endl << "Press 'q' to close each windows ... " << endl;
  waitKey(0);
  window.spin();
  return 0;
}
