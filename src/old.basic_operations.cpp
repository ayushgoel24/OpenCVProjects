#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d.hpp>



#include <iostream>
#include <stdio.h>































//http://docs.opencv.org/3.0-beta/doc/py_tutorials/py_calib3d/py_pose/py_pose.html
void draw_xyz_frame_over_image(cv::Mat image,cv::Point2f reference_point,std::vector<cv::Point2f >end_points ,int thickness=5)
{
    cv::line(image,reference_point, end_points.at(0),cv::Scalar(255,0,0),thickness);
    cv::line(image,reference_point,end_points.at(1) ,cv::Scalar(0,255,0),thickness);
    cv::line(image,reference_point,end_points.at(2) ,cv::Scalar(0,0,255),thickness);
}

void chessboard_pose_estimation(int argc, char **argv)
{
    cv::VideoCapture camera(0);
    cv::Mat image,image_gray;
    std::vector<cv::Point2f> chess_board_corners;
    int width, height;
    width=9;
    height=6;
    double square_size=0.025;
    std::vector<cv::Point3f> chess_board_point;
    cv::Size board_size=cv::Size (width, height);
    bool found;

    cv::Mat rvec(1,3,cv::DataType<double>::type);
    cv::Mat tvec(1,3,cv::DataType<double>::type);
    cv::Mat axis=cv::Mat::eye(3,3,CV_32F)*3;
    std::vector<cv::Point2f> axis_end_point;


    for(std::size_t i=0;i<width;i++)
    {
        for(std::size_t j=0;j<height;j++)
        {
            cv::Point3f chess_board_3d_point;
            chess_board_3d_point.x=i*square_size;
            chess_board_3d_point.y=j*square_size;
            chess_board_3d_point.z=0;
            chess_board_point.push_back(chess_board_3d_point);
//            std::cout<<"x: "<<i*square_size <<std::endl;
//            std::cout<<"y: "<<j*square_size <<std::endl;
        }
    }



    std::string camera_calibration_path="front_webcam.yml";
    cv::FileStorage fs(camera_calibration_path,cv::FileStorage::READ);
    cv::Mat camera_matrix, distortion_coefficient;
    fs["camera_matrix"]>>camera_matrix;
    fs["distortion_coefficients"]>>distortion_coefficient;

    while(true)
    {
        camera>>image;
        cv::cvtColor(image,image_gray,cv::COLOR_BGR2GRAY);
        int key=cv::waitKey(30);
        if( (char)key==(char)27 )
        {
            break;
        }
        found=cv::findChessboardCorners(image,board_size,chess_board_corners);
        if(found)
        {
//            std::cout<<"found" <<std::endl;
            TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::MAX_ITER|cv::TermCriteria::EPS, 30, 0.1);
            cv::cornerSubPix(image_gray,chess_board_corners, Size(11, 11), Size(-1, -1), criteria);

            //cv::drawChessboardCorners(image,board_size,chess_board_corners,found);
            cv::solvePnPRansac(chess_board_point,chess_board_corners,camera_matrix,distortion_coefficient,rvec, tvec);
            std::cout<<tvec <<std::endl;
            std::cout<<"-----------------------------------------" <<std::endl;
            cv::projectPoints(axis,rvec,tvec,camera_matrix,distortion_coefficient,axis_end_point);

            draw_xyz_frame_over_image(image,chess_board_corners.at(0), axis_end_point );
        }

        cv::imshow("pose",image);
    }

    return;
}





//http://paulbourke.net/miscellaneous/correlate/
//http://www.ee.ic.ac.uk/hp/staff/dmb/courses/E1Fourier/00800_Correlation.pdf
//https://docs.opencv.org/3.2.0/de/da9/tutorial_template_matching.html

/*
CV_TM_SQDIFF -> sum of square difference (ssd)
CV_TM_SQDIFF_NORMED ->  squared difference normalized
CV_TM_CCORR -> cross correlation
CV_TM_CCORR_NORMED cross correlation normalized
CV_TM_CCOEFF -> cross correlation coefficient
CV_TM_CCOEFF_NORMED -> cross correlation coefficient normalized

Pearson correlation coefficient ??
*/

/// Global Variables
Mat img; Mat templ; Mat result;
char* image_window = "Source Image";
char* result_window = "Result window";

int match_method;
int max_Trackbar = 5;


//void MatchingMethod( int, void* )
//{
//    /// Source image to display
//    Mat img_display;
//    img.copyTo( img_display );

//    /// Create the result matrix
//    int result_cols =  img.cols - templ.cols + 1;
//    int result_rows = img.rows - templ.rows + 1;

//    result.create( result_cols, result_rows, CV_32FC1 );

//    /// Do the Matching and Normalize
//    matchTemplate( img, templ, result, match_method );
//    normalize( result, result, 0, 1, NORM_MINMAX, -1, Mat() );

//    /// Localizing the best match with minMaxLoc
//    double minVal; double maxVal; Point minLoc; Point maxLoc;
//    Point matchLoc;

//    minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, Mat() );

//    /// For SQDIFF and SQDIFF_NORMED, the best matches are lower values. For all the other methods, the higher the better
//    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
//    { matchLoc = minLoc; }
//    else
//    { matchLoc = maxLoc; }

//    /// Show me what you got
//    rectangle( img_display, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );
//    rectangle( result, matchLoc, Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), Scalar::all(0), 2, 8, 0 );

//    imshow( image_window, img_display );
//    imshow( result_window, result );

//    return;
//}



