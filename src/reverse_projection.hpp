/////////////////////////////////////////implementing OpenCV undistortPoints///////////////////////////////////////
 
    cv::Mat imagePointMatrix=cv::Mat_<double>(3,1);
    cv::Mat imageRayMatrix=cv::Mat_<double>(3,1);
 
    std::cout<<"===============Inverse of Camera Matrix(OpenCV)==========================="<<std::endl;
    std::cout<<cameraMatrix.inv()<<std::endl;
 
    std::cout<<"=====================Inverse of Camera Matrix====================="<<std::endl;
 
    cv::Mat inv=(cv::Mat_<double>(3,3)<< cameraMatrix.at<double>(1,1)  ,0,-cameraMatrix.at<double>(1,1)*cameraMatrix.at<double>(0,2)
                 ,0,cameraMatrix.at<double>(0,0),-cameraMatrix.at<double>(1,2)*cameraMatrix.at<double>(0,0)
                 ,0,0,cameraMatrix.at<double>(0,0)*cameraMatrix.at<double>(1,1));
    std::cout<<inv/(cameraMatrix.at<double>(1,1)*cameraMatrix.at<double>(0,0))<<std::endl;
 
    std::cout<<"=====================Computed Rays===================== "<<std::endl;
 
 
    for(std::size_t i=0;i<imagePointsInCamera.size();i++)
    {
        imagePointMatrix.at<double>(0,0)=imagePointsInCamera[i].x;
        imagePointMatrix.at<double>(1,0)=imagePointsInCamera[i].y;
        imagePointMatrix.at<double>(2,0)=1.0;
        imageRayMatrix=cameraMatrix.inv()*imagePointMatrix;
 
 
 
        std::cout<<imageRayMatrix <<std::endl;
 
 
    }
/*
    (u,v) is the input point, (u', v') is the output point
    camera_matrix=[fx 0 cx; 0 fy cy; 0 0 1]
    P=[fx' 0 cx' tx; 0 fy' cy' ty; 0 0 1 tz]
    x" = (u - cx)/fx
    y" = (v - cy)/fy
    (x',y') = undistort(x",y",dist_coeffs)
    [X,Y,W]T = R*[x' y' 1]T
    x = X/W, y = Y/W
    only performed if P=[fx' 0 cx' [tx]; 0 fy' cy' [ty]; 0 0 1 [tz]] is specified
    u' = x*fx' + cx'
    v' = y*fy' + cy',
*/
 
    std::vector<cv::Point2d> imagePointsInCameraNormalizedCoordinate;
    cv::undistortPoints(imagePointsInCamera,imagePointsInCameraNormalizedCoordinate,cameraMatrix,distortionCoefficient);
    std::vector<cv::Point3d> imagePointsInCameraNormalizedCoordinateHomogeneous;
    cv::convertPointsToHomogeneous(imagePointsInCameraNormalizedCoordinate,imagePointsInCameraNormalizedCoordinateHomogeneous);
    cv::Mat imagePointsInCameraNormalizedCoordinateHomogeneousMatrix=cv::Mat(imagePointsInCameraNormalizedCoordinateHomogeneous);
    imagePointsInCameraNormalizedCoordinateHomogeneousMatrix=imagePointsInCameraNormalizedCoordinateHomogeneousMatrix.reshape(1).t();
    double a,b,c;
    std::cout<<"=====================OpenCV Rays=====================" <<std::endl;
 
    for(int i=0;i<imagePointsInCameraNormalizedCoordinateHomogeneousMatrix.cols;i++)
    {
        a=imagePointsInCameraNormalizedCoordinateHomogeneousMatrix.at<double>(0,i);
        b=imagePointsInCameraNormalizedCoordinateHomogeneousMatrix.at<double>(1,i);
        c=imagePointsInCameraNormalizedCoordinateHomogeneousMatrix.at<double>(2,i);
        std::cout<<a <<std::endl;
        std::cout<<b <<std::endl;
        std::cout<<c <<std::endl;
        directionRays.push_back(cv::Point3d(a,b,c));
 
    }
/*
The inverse of a 3x3 matrix:
| a11 a12 a13 |-1
| a21 a22 a23 |    =  1/DET * A^-1
| a31 a32 a33 |
 
with A^-1  =
 
|  a33a22-a32a23  -(a33a12-a32a13)   a23a12-a22a13 |
|-(a33a21-a31a23)   a33a11-a31a13  -(a23a11-a21a13)|
|  a32a21-a31a22  -(a32a11-a31a12)   a22a11-a21a12 |
 
and DET  =  a11(a33a22-a32a23) - a21(a33a12-a32a13) + a31(a23a12-a22a13)
 
Camera Matrix:
 
|fx 0 cx|
|0 fy cy|
|0 0  1 |
 
Rays are  A^-1*p:
 
 1    |fy 0   -fycx|  |u|
----- |0  fx -cy*fx| *|v| = [ (u- cx)/fx, (v-cx)/fy, 1]
fx*fy |0  0   fy*fx|  |1|
 
*/
