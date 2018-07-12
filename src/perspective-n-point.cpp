#include <opencv2/opencv.hpp>
/*

Perspective-n-Point:
Perspective-n-Point is the problem of estimating the pose of a calibrated camera given a
set of n 3D points in the world and their corresponding 2D projections in the image.
*/
void pnp_example(int argc, char ** argv)
{
/*The data for this example are coming from pnp.jpg
so the world coordinate system is corner of the room next to door and other 4 points are green dots in the image

http://stackoverflow.com/questions/12299870/computing-x-y-coordinate-3d-from-image-point
https://en.wikipedia.org/wiki/Perspective-n-Point
*/

    std::vector<cv::Point2f> imagePoints;
    std::vector<cv::Point3f> objectPoints;
    //img points are green dots in the picture
    imagePoints.push_back(cv::Point2f(271.,109.));
    imagePoints.push_back(cv::Point2f(65.,208.));
    imagePoints.push_back(cv::Point2f(334.,459.));
    imagePoints.push_back(cv::Point2f(600.,225.));

    //object points are measured in millimeters because calibration is done in mm also
    objectPoints.push_back(cv::Point3f(0., 0., 0.));
    objectPoints.push_back(cv::Point3f(-511.,2181.,0.));
    objectPoints.push_back(cv::Point3f(-3574.,2354.,0.));
    objectPoints.push_back(cv::Point3f(-3400.,0.,0.));

    cv::Mat rvec(1,3,cv::DataType<double>::type);
    cv::Mat tvec(1,3,cv::DataType<double>::type);
    cv::Mat rotationMatrix(3,3,cv::DataType<double>::type);

    cv::FileStorage fs(argv[1],cv::FileStorage::READ);

    cv::Mat camera_matrix,distortion_coefficients;

    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> distortion_coefficients;


    cv::solvePnP(objectPoints, imagePoints, camera_matrix, distortion_coefficients, rvec, tvec);

}


/*

projection matrix contains the intrinsic parameter matrix of the camera multiplied by the extrinsic parameters matrix of the matrix.
The extrinsic parameter matrix itself provides the roto-translation of the camera frame with respect to the world frame.

The projection matrix P (dimensions 3 x 4) contains the extrinsic and intrinsic camera parameters and is
intended to be used to map homogenous world coordinates (4 x 1) to homogeneous camera coordinates (3 x 1)


Multiple View Geometry in Computer Vision  156 eg. 6.8
p projection matrix
K intrincis camera (camera matrix)
R rotation w.r.t world
T translation w.r.t world

P = K [ R | t ]

*/
void projectionMatrix_example()
{

}



int main(int argc, char** argv)
{

    pnp_example(argc, argv);
}

