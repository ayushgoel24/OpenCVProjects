#include "csv.h"
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>

std::vector<cv::Point3d>
readPoints(std::string pathToPointFile = "../data/points.csv") {
  io::CSVReader<3> in(pathToPointFile);
  in.read_header(io::ignore_extra_column, "x", "y", "z");
  double x, y, z;
  std::vector<cv::Point3d> objectPoints;

  while (in.read_row(x, y, z)) {
    objectPoints.push_back(cv::Point3d(x, y, z));
  }

  std::cout << "OpenCV coordinate:\n" << std::endl;

  std::cout << "                 Z" << std::endl;
  std::cout << "                ▲" << std::endl;
  std::cout << "               /" << std::endl;
  std::cout << "              /" << std::endl;
  std::cout << "             /1 2 3 4     X" << std::endl;
  std::cout << "            |------------ ⯈" << std::endl;
  std::cout << "           1|" << std::endl;
  std::cout << "           2|" << std::endl;
  std::cout << "           3|" << std::endl;
  std::cout << "           4|" << std::endl;
  std::cout << "            | Y" << std::endl;
  std::cout << "            ⯆" << std::endl;

  std::cout << "\npoints in 3d world:\n" << std::endl;

  for (const auto p : objectPoints)
    std::cout << p << std::endl;

  return objectPoints;
}

cv::Mat rotationMatrixFromRollPitchYaw(double alpha, double beta,
                                       double gamma) {
  /*
      yaw:
          A yaw is a counterclockwise rotation of alpha about the  z-axis. The
      rotation matrix is given by

          R_z

          |cos(alpha) -sin(alpha) 0|
          |sin(alpha)   cos(alpha) 0|
          |    0            0     1|

      pitch:
          R_y
          A pitch is a counterclockwise rotation of  beta about the  y-axis. The
      rotation matrix is given by

          |cos(beta)  0   sin(beta)|
          |0          1       0    |
          |-sin(beta) 0   cos(beta)|

      roll:
          A roll is a counterclockwise rotation of  gamma about the  x-axis. The
      rotation matrix is given by
          R_x
          |1          0           0|
          |0 cos(gamma) -sin(gamma)|
          |0 sin(gamma)  cos(gamma)|
  */

  cv::Mat R_z = (cv::Mat_<double>(3, 3) << cos(alpha), -sin(alpha), 0,
                 sin(alpha), cos(alpha), 0, 0, 0, 1);

  cv::Mat R_y = (cv::Mat_<double>(3, 3) << cos(beta), 0, sin(beta), 0, 1, 0,
                 -sin(beta), 0, cos(beta));

  cv::Mat R_x = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, cos(gamma), -sin(gamma),
                 0, sin(gamma), cos(gamma));

  return R_z * R_y * R_x;
}

cv::Mat findFundamentalMatrix(std::vector<cv::Point2d> &imagePointsLeftCamera,
                           std::vector<cv::Point2d> &imagePointsRightCamera) {
  std::vector<cv::Point3d> imagePointsLeftCameraHomogeneous,
      imagePointsRightCameraHomogeneous;
  cv::convertPointsToHomogeneous(imagePointsLeftCamera,
                                 imagePointsLeftCameraHomogeneous);
  cv::convertPointsToHomogeneous(imagePointsRightCamera,
                                 imagePointsRightCameraHomogeneous);
  /*

     ┌       ┐ ┌f11  f12  f13┐ ┌u┐
     |u` v` 1|*|f21  f22  f23|*|v|=0
     └       ┘ └f31  f32  f33┘ └1┘

     ┌u'1u1   u'1v1   u'1   v'1u1   v'1v1   v'1   u1   v1   1┐   ┌f11┐
     |u'2u2   u'2v2   u'2   v'2u2   v'2v2   v'2   u2   v2   1|   |f12|
     |u'3u3   u'3v3   u'3   v'3u3   v'3v3   v'3   u3   v3   1|   |f13|
     |u'4u4   u'4v4   u'4   v'4u4   v'4v4   v'4   u4   v4   1|   |f21|
     |u'5u5   u'5v5   u'5   v'5u5   v'5v5   v'5   u5   v5   1| * |f22|=0
     |u'6u6   u'6v6   u'6   v'6u6   v'6v6   v'6   u6   v6   1|   |f23|
     |u'7u7   u'7v7   u'7   v'7u7   v'7v7   v'7   u7   v7   1|   |f31|
     └u'8u8   u'8v8   u'8   v'8u8   v'8v8   v'8   u8   v8   1┘   |f32|
                                                                 └f33┘
  */
  double u_prime, v_prime, u, v;
  cv::Mat A = cv::Mat_<double>(imagePointsLeftCamera.size(), 9);
  for (std::size_t i = 0; i < imagePointsLeftCamera.size(); i++) {
    u_prime = imagePointsLeftCamera.at(i).x;
    v_prime = imagePointsLeftCamera.at(i).y;

    u = imagePointsRightCamera.at(i).x;
    v = imagePointsRightCamera.at(i).y;

    A.at<double>(i, 0) = u_prime * u;
    A.at<double>(i, 1) = u_prime * v;
    A.at<double>(i, 2) = u_prime;
    A.at<double>(i, 3) = v_prime * u;
    A.at<double>(i, 4) = v_prime * v;
    A.at<double>(i, 5) = v_prime;
    A.at<double>(i, 6) = u;
    A.at<double>(i, 7) = v;
    A.at<double>(i, 8) = 1;
  }

  cv::Mat U, SingularValuesVector, VT;
  cv::Mat SigmaMatrix = cv::Mat::zeros(A.rows, A.cols, CV_64F);
  cv::SVD::compute(A.clone(), SingularValuesVector, U, VT);

  ///////////////// Building Square Matrix U  ////////////////////

  cv::Mat completeU = cv::Mat_<double>(U.rows, U.rows);
  cv::Mat missingElementsOfU = cv::Mat::zeros(U.rows, U.rows - U.cols, CV_64F);
  cv::hconcat(U, missingElementsOfU, completeU);

  /////////////////  Building Sigma Matrix /////////////////

  cv::Mat completeSigma = cv::Mat::zeros(completeU.cols, VT.rows, CV_64F);
  for (int i = 0; i < SingularValuesVector.rows; i++) {
    completeSigma.at<double>(i, i) = SingularValuesVector.at<double>(i, 0);
  }

  ///////////////// Checking A=completeU*completeSigma*Vt /////////////////

  std::cout << "checking A-U*Sigma*VT=0" << std::endl;
  std::cout << cv::sum(A - completeU * completeSigma * VT).val[0] << std::endl;

  ///////////////// Building F Matrix From F vector /////////////////

  cv::Mat F_vec = VT.col(VT.cols - 1);
  std::cout << F_vec.cols << std::endl;
  cv::Mat F = cv::Mat(3, 3, cv::DataType<double>::type);

  F.at<double>(0, 0) = F_vec.at<double>(0, 0);
  F.at<double>(0, 1) = F_vec.at<double>(1, 0);
  F.at<double>(0, 2) = F_vec.at<double>(2, 0);
  F.at<double>(1, 0) = F_vec.at<double>(3, 0);
  F.at<double>(1, 1) = F_vec.at<double>(4, 0);
  F.at<double>(1, 2) = F_vec.at<double>(5, 0);
  F.at<double>(2, 0) = F_vec.at<double>(6, 0);
  F.at<double>(2, 1) = F_vec.at<double>(7, 0);
  F.at<double>(2, 2) = F_vec.at<double>(8, 0);

  ///////////////// Computing SVD of F /////////////////

  cv::SVD::compute(F.clone(), SingularValuesVector, U, VT);
  std::cout << "F singular values" << std::endl;
  std::cout << SingularValuesVector << std::endl;

  ///////////////// Setting The Smallest Eigen Value to Zero /////////////////
  SingularValuesVector.at<double>(SingularValuesVector.rows - 1, 0) = 0;

  ///////////////// Building U (Building Square Matrix U) /////////////////

  completeU = cv::Mat_<double>(U.rows, U.rows);
  missingElementsOfU = cv::Mat::zeros(U.rows, U.rows - U.cols, CV_64F);
  cv::hconcat(U, missingElementsOfU, completeU);

  ///////////////// Building Sigma Matrix /////////////////

  completeSigma = cv::Mat::zeros(completeU.cols, VT.rows, CV_64F);
  for (int i = 0; i < SingularValuesVector.rows; i++) {
    completeSigma.at<double>(i, i) = SingularValuesVector.at<double>(i, 0);
  }
  ///////////////// Building New F matrix /////////////////

  cv::Mat NewF = completeU * completeSigma * VT;
//   std::cout << "Fundamental Matrix is:" << std::endl;
//   std::cout << NewF << std::endl;

//   cv::Ptr<cv::Formatter> formatMat =
//       cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
//   formatMat->set64fPrecision(3);
//   formatMat->set32fPrecision(3);

//   std::cout << std::endl << formatMat->format(NewF) << std::endl;

  return NewF;
}

void project3DPoint() {

  /*

  OpenCV camera coordinate:

                    Z
                  ▲
                 /
                /
               /1 2 3 4     x or u means column
              |------------ ⯈
             1|
             2|
             3|
             4|
              | y or v means row
              ⯆




  In OpenCV, Point(x=column,y=row). For instance the point in the following
  image can be accessed with

      X
      --------column---------►
      | Point(0,0) Point(1,0) Point(2,0) Point(3,0)
      | Point(0,1) Point(1,1) Point(2,1) Point(3,1)
      | Point(0,2) Point(1,2) Point(2,2) Point(3,2)
    y |
     row
      |
      |
      ▼

      However if you access an image directly, the order is
  mat.at<type>(row,column). So the following will return the same value:
      mat.at<type>(row,column)
      mat.at<type>(cv::Point(column,row))

      X
      --------column---------►
      | mat.at<type>(0,0) mat.at<type>(0,1) mat.at<type>(0,2) mat.at<type>(0,3)
      | mat.at<type>(1,0) mat.at<type>(1,1) mat.at<type>(1,2) mat.at<type>(1,3)
      | mat.at<type>(2,0) mat.at<type>(2,1) mat.at<type>(2,2) mat.at<type>(2,3)
    y |
     row
      |
      |
      ▼



  The parameters fx=f*mx  and fy=f*my  where mx=1/width and my=1/height  meaning
  size of 1 pixel in x and y

  mx=1/width
  my=1/height

  cx=Width/2;
  cy=Height/2 ;

  fx=f*mx

  k=[fx  0  cx
     0  fy  cy
     0  0   1 ]

                  Z
                  ▲
                 /
                /
               /1 2 3 4     X
              |------------ ⯈
             1|
             2|
             3|
             4|
              | Y
              ⯆


  mat.at<type>(row,column)
  mat.at<type>(cv::Point(column,row))

  */

  ///////////////// camera intrinsic /////////////////
  int numberOfPixelInHeight, numberOfPixelInWidth;
  double heightOfSensor, widthOfSensor;
  double focalLength = 0.1;
  double mx, my, cx, cy;

  numberOfPixelInHeight = 480;
  numberOfPixelInWidth = 640;

  heightOfSensor = 10;
  widthOfSensor = 10;

  mx = (numberOfPixelInWidth) / widthOfSensor;
  my = (numberOfPixelInHeight) / heightOfSensor;

  cx = (numberOfPixelInWidth) / 2;
  cy = (numberOfPixelInHeight) / 2;

  cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << focalLength * mx, 0, cx, 0,
                          focalLength * my, cy, 0, 0, 1);

  std::cout << "camera intrinsic:\n" << cameraMatrix << std::endl;

  cv::Mat distortionCoefficient = (cv::Mat_<double>(5, 1) << 0, 0, 0, 0, 0);
  std::cout << "camera distortion coefficient:\n"
            << distortionCoefficient << std::endl;

  ///////////////// cameras extrinsic /////////////////
  /*


                  Z                        Z
                  ▲                         ▲
                 /                           \
                /                             \
               /1 2 3 4     X                  \ 1 2 3 4
  Left Cam   |------------ ⯈                   |------------ ⯈Right cam
            1|                               1 |
            2|                               2 |
            3|                               3 |
           Y |                               Y |
             ⯆                                ⯆

  We set the world ref frame on the left camera and translate the right camera
  rotate it around Y axis (pitch)

  */

  cv::Mat leftCameraRotation = cv::Mat::eye(3, 3, CV_64FC1);
  cv::Mat leftCameraTranslation = (cv::Mat_<double>(1, 3) << 0, 0, 0);

  double alpha, beta, gamma;

  alpha = 0;
  // beta = -M_PI / 36;
  beta = 0;
  gamma = 0;

  cv::Mat rightCameraRotation =
      rotationMatrixFromRollPitchYaw(alpha, beta, gamma);
  cv::Mat rightCameraTranslation = (cv::Mat_<double>(1, 3) << 0.0, 0.0, 0.0);

  ///////////////// 3D points from world /////////////////

  std::vector<cv::Point3d> objectPoints = readPoints("../data/points.csv");

  std::vector<cv::Point2d> projectedPointsInLeftCamera,
      projectedPointsInRightCamera;

  ///////////////// projecting 3D points into camera /////////////////

  cv::projectPoints(objectPoints, leftCameraRotation, leftCameraTranslation,
                    cameraMatrix, distortionCoefficient,
                    projectedPointsInLeftCamera);

  cv::projectPoints(objectPoints, rightCameraRotation, rightCameraTranslation,
                    cameraMatrix, distortionCoefficient,
                    projectedPointsInRightCamera);

  std::cout << "projected point in left camera" << std::endl;
  for (const auto p : projectedPointsInLeftCamera)
    std::cout << "row: " << p.y << ","
              << " column: " << p.x << std::endl;

  std::cout << "projected point in right camera" << std::endl;
  for (const auto p : projectedPointsInRightCamera)
    std::cout << "row: " << p.y << ","
              << " column: " << p.x << std::endl;

  ///////////////// saving the image  /////////////////
  double row, col;

  std::string fileName;

  cv::Mat cameraImageRight =
      cv::Mat::zeros(numberOfPixelInHeight, numberOfPixelInWidth, CV_8UC1);
  cv::line(cameraImageRight, cv::Point2d(numberOfPixelInWidth / 2, 0),
           cv::Point2d(numberOfPixelInWidth / 2, numberOfPixelInHeight),
           cv::Scalar(255, 255, 255));
  cv::line(cameraImageRight, cv::Point2d(0, numberOfPixelInHeight / 2),
           cv::Point2d(numberOfPixelInWidth, numberOfPixelInHeight / 2),
           cv::Scalar(255, 255, 255));
  for (std::size_t i = 0; i < projectedPointsInRightCamera.size(); i++) {
    col = int(projectedPointsInRightCamera.at(i).x);
    row = int(projectedPointsInRightCamera.at(i).y);
    // std::cout<<row <<"," <<col  <<std::endl;
    cameraImageRight.at<char>(int(row), int(col)) = char(255);
  }
  fileName = std::string("right_cam_image_") + std::to_string(focalLength) +
             std::string("_.jpg");
  cv::imwrite(fileName, cameraImageRight);

  cv::Mat cameraImageLeft =
      cv::Mat::zeros(numberOfPixelInHeight, numberOfPixelInWidth, CV_8UC1);
  cv::line(cameraImageLeft, cv::Point2d(numberOfPixelInWidth / 2, 0),
           cv::Point2d(numberOfPixelInWidth / 2, numberOfPixelInHeight),
           cv::Scalar(255, 255, 255));
  cv::line(cameraImageLeft, cv::Point2d(0, numberOfPixelInHeight / 2),
           cv::Point2d(numberOfPixelInWidth, numberOfPixelInHeight / 2),
           cv::Scalar(255, 255, 255));
  for (std::size_t i = 0; i < projectedPointsInLeftCamera.size(); i++) {
    col = int(projectedPointsInLeftCamera.at(i).x);
    row = int(projectedPointsInLeftCamera.at(i).y);
    // std::cout<<row <<"," <<col  <<std::endl;
    cameraImageLeft.at<char>(int(row), int(col)) = char(255);
  }
  fileName = std::string("left_cam_image_") + std::to_string(focalLength) +
             std::string("_.jpg");
  cv::imwrite(fileName, cameraImageLeft);

  findFundamentalMatrix(projectedPointsInLeftCamera,
                        projectedPointsInRightCamera);
}

int main(int argc, char **argv11) { project3DPoint(); }
