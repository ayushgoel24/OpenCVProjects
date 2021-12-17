#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

void FASTFeatureDetector(cv::Mat &image, std::vector<cv::KeyPoint> &key_points)
{
//    //cv::FeatureDetector
//    std::string path_to_image="../images/Hough_Lines_Tutorial_Original_Image.jpg";
//    cv::Mat image;
//    image=cv::imread(path_to_image);
//    std::vector<cv::KeyPoint> key_points;
    key_points.clear();
    int fast_threshold = 20;
    bool nonmaxSuppression = true;
    cv::FAST(image,key_points,fast_threshold,nonmaxSuppression);
//    cv::drawKeypoints(image,key_points,image);
//    cv::imshow("Fast Keypoints",image);
//    cv::waitKey();
}

void temp()
{
    cv::Mat image;
    std::string path_to_image="../images/stereo_calibration/left01.jpg";
    image=cv::imread(path_to_image);
    std::vector<cv::KeyPoint> key_points;
    FASTFeatureDetector(image, key_points);
    cv::drawKeypoints(image,key_points,image);
    cv::imshow("Fast Keypoints",image);
    cv::waitKey();
}

void Camera()
{

    cv::VideoCapture webCam(0); // open the default camera
    webCam.set(cv::CAP_PROP_FRAME_WIDTH,640);
    webCam.set(cv::CAP_PROP_FRAME_HEIGHT,480);
    std::vector<cv::KeyPoint> key_points;
    if(!webCam.isOpened())  // check if we succeeded
        return;

    cv::namedWindow("camera",1);
    for(;;)
    {
        cv::Mat frame;
        webCam >> frame; // get a new frame from camera
        std::vector<cv::Point2f> points;



        cv::cvtColor(frame,frame, cv::COLOR_BGR2RGB );
        FASTFeatureDetector(frame,  key_points);
//        cv::KeyPoint::convert(key_points, points, std::vector<int>());
        cv::drawKeypoints(frame,key_points,frame);


        cv::imshow("camera", frame);
        if(cv::waitKey(1) >= 0)
            break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return ;
}

int main(int argc, char** argv)
{
    //FASTFeatureDetector();
    Camera();
//     temp();
}
