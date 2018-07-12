#include <opencv2/opencv.hpp>






const char *windowDisparity = "Disparity";

cv::Mat imgLeft;
cv::Mat imgRight;


cv::Mat imgDisparity16S;
cv::Mat imgDisparity8U;

int ndisparities= 16*5;
int SADWindowSize= 21;
double minVal; double maxVal;


cv::StereoBM sbm( cv::StereoBM::NARROW_PRESET,ndisparities,SADWindowSize );

void on_trackbar( int, void* )
{

    std::cout <<"ndisparities: " <<16*ndisparities <<std::endl;
    std::cout <<"SADWindowSize: " <<SADWindowSize <<std::endl;
    if(SADWindowSize%2 !=0 && SADWindowSize>4)
    {
        sbm.init(cv::StereoBM::NARROW_PRESET,16*ndisparities ,SADWindowSize);
        sbm( imgLeft, imgRight, imgDisparity16S, CV_16S );
        minMaxLoc( imgDisparity16S, &minVal, &maxVal );
        imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));

        cv::namedWindow( windowDisparity, cv::WINDOW_NORMAL );
        cv::imshow( windowDisparity, imgDisparity8U );
    }



}


//For an in-depth discussion of the block matching algorithm, see pages 438-444 of Learning OpenCV.
//https://github.com/opencv/opencv/blob/2.4/samples/cpp/tutorial_code/calib3d/stereoBM/SBM_Sample.cpp
int disparity_map_using_sbm_example(int argc, char ** argv)
{

    //char* n_argv[] = { "stereo_block_matching", "../images/stereo_vision/tsucuba_left.png", "../images/stereo_vision/tsucuba_right.png"};

    char* n_argv[] = { "stereo_block_matching", "rect_left01.jpg", "rect_right01.jpg"};

    int length = sizeof(n_argv)/sizeof(n_argv[0]);

    argc=length;
    argv = n_argv;


    imgLeft = cv::imread( argv[1], CV_LOAD_IMAGE_GRAYSCALE );
    imgRight = cv::imread( argv[2], CV_LOAD_IMAGE_GRAYSCALE );



    //-- And create the image in which we will save our disparities
    imgDisparity16S = cv::Mat( imgLeft.rows, imgLeft.cols, CV_16S );
    imgDisparity8U = cv::Mat( imgLeft.rows, imgLeft.cols, CV_8UC1 );


    //-- 2. Call the constructor for StereoBM
    //ndisparities must be multiple of 8


    //-- 3. Calculate the disparity image
    sbm( imgLeft, imgRight, imgDisparity16S, CV_16S );


    //-- Check its extreme values


    minMaxLoc( imgDisparity16S, &minVal, &maxVal );

    printf("Min disp: %f Max value: %f \n", minVal, maxVal);

    //-- 4. Display it as a CV_8UC1 image
    imgDisparity16S.convertTo( imgDisparity8U, CV_8UC1, 255/(maxVal - minVal));

    cv::namedWindow( windowDisparity, cv::WINDOW_NORMAL );
    cv::imshow( windowDisparity, imgDisparity8U );


    //Create trackbars in "Control" window
    ndisparities=ndisparities/16;
    cv::createTrackbar("ndisparities (multipled by 16)", windowDisparity, &ndisparities, 20,on_trackbar); //ndisparities (0 - 20)
    cv::createTrackbar("SADWindowSize (must be odd, be within 5..255) ", windowDisparity, &SADWindowSize, 255,on_trackbar); //SADWindowSize(0 - 100)



    //-- 5. Save the image
    //imwrite("SBM_sample.png", imgDisparity16S);

//    cv::reprojectImageTo3D(imgDisparity8U,)

    cv::waitKey(0);

    return 0;


}


void disparity_map_using_sgbm_example(int argc, char ** argv)
{


    char* n_argv[] = { "stereo_block_matching", "../images/stereo_vision/tsucuba_left.png", "../images/stereo_vision/tsucuba_right.png"};
    int length = sizeof(n_argv)/sizeof(n_argv[0]);
    argc=length;
    argv = n_argv;

    cv::Mat img1, img2, g1, g2;
    cv::Mat disp, disp8;
    img1 = cv::imread(argv[1]);
    img2 = cv::imread(argv[2]);
    cv::cvtColor(img1, g1, CV_BGR2GRAY);
    cv::cvtColor(img2, g2, CV_BGR2GRAY);


    cv::StereoSGBM sbm;
    sbm.SADWindowSize = 3;
    sbm.numberOfDisparities = 144;
    sbm.preFilterCap = 63;
    sbm.minDisparity = -39;
    sbm.uniquenessRatio = 10;
    sbm.speckleWindowSize = 100;
    sbm.speckleRange = 32;
    sbm.disp12MaxDiff = 1;
    sbm.fullDP = false;
    sbm.P1 = 216;
    sbm.P2 = 864;
    sbm(g1, g2, disp);

    normalize(disp, disp8, 0, 255, CV_MINMAX, CV_8U);

    imshow("left", img1);
    imshow("right", img2);
    imshow("disp", disp8);

    cv::waitKey(0);

//    cv::finds

}


int main(int argc, char** argv)
{
//    disparity_map_using_sgbm_example(argc, argv);
    disparity_map_using_sbm_example(argc, argv);
}


