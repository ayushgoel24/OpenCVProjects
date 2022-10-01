#include <opencv4/opencv2/opencv.hpp>




cv::Mat rotationMatrixFromRollPitchYaw(double roll, double pitch, double yaw)
{
/*
Great Tutorial:
http://planning.cs.uiuc.edu/node102.html
http://euclideanspace.com/maths/geometry/rotations/conversions/index.htm

Tait–Bryan angles: Z1Y2X3 in the wiki page:
https://en.wikipedia.org/wiki/Euler_angles
  yaw:
      A yaw is a counterclockwise rotation of alpha about the  z-axis. The
  rotation matrix is given by

      R_z

      |cos(alpha) -sin(alpha) 0|
      |sin(apha)   cos(alpha) 0|
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



      It is important to note that   R_z R_y R_x performs the roll first, then the pitch, and finally the yaw
      Roration matrix: R_z*R_y*R_x

*/


    cv::Mat R_z = (cv::Mat_<double>(3,3)<<
      cos(yaw), -sin(yaw), 0
      ,sin(yaw), cos(yaw), 0
      ,0, 0, 1);

    cv::Mat R_y = (cv::Mat_<double>(3,3)<<
      cos(pitch), 0,sin(pitch)
      ,0,1,0
      ,-sin(pitch),0,cos(pitch) );

    cv::Mat R_x =( cv::Mat_<double>(3,3)<<
      1,0,0
      ,0, cos(roll), -sin(roll)
      ,0, sin(roll),  cos(roll));


    // std::cout << std::setprecision(2) << std::fixed;
    // std::cout.setf(std::ios::fixed, std::ios::floatfield);
    // std::cout.setf(std::ios::showpoint);

    //std::cout<<R_z<<"\n"  <<R_y<<"\n" <<R_x <<std::endl;

    return R_z*R_y*R_x;

}


void rodrigueRotationMatrix()
{
    // https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga61585db663d9da06b68e70cfbf6a1eac

    //Converts a rotation matrix to a rotation vector.

    cv::Mat rvec = cv::Mat_<double>(3,1);
    cv::Mat rotationMatrix = cv::Mat_<double>(3,3);


    double roll,  pitch,  yaw;
    roll=M_PI/2;
    pitch=M_PI/2;
    yaw=0;//M_PI/6;


    rotationMatrix= rotationMatrixFromRollPitchYaw(roll, pitch,  yaw);



    cv::Ptr<cv::Formatter> fmt = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
    

    fmt->set64fPrecision(3);
    fmt->set32fPrecision(3);
    

    std::cout<<"Rotation Matrix:\n" <<fmt->format(rotationMatrix) <<std::endl;


    cv::Rodrigues(rotationMatrix,rvec);
    std::cout<<"Calculated Rodrigues vector:\n" <<rvec <<std::endl;

    //Converts a rotation vector  to a rotation matrix to a or vice versa.
    rotationMatrix=cv::Mat::zeros(3,3,CV_64FC1);
    cv::Rodrigues(rvec,rotationMatrix);
    std::cout<<"Calculated Rotation Matrix from Rodrigues vector:\n" <<rotationMatrix <<std::endl;
}



int main(int argc, char * argv[])
{
    rodrigueRotationMatrix();
    return 0;
}