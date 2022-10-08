#include <opencv2/opencv.hpp>




void createAffineMatrixExample(int argc, char **argv)
{
    cv::Mat src, warp_dst, warp_rotate_dst;
    src = cv::imread( argv[1], cv::IMREAD_ANYCOLOR  );
    
    cv::Mat rot_mat( 2, 3, CV_32FC1 );
    cv::Mat warp_mat( 2, 3, CV_32FC1 );

    //cv::remap();
    //convertMaps

    //https://docs.opencv.org/3.4/d1/da0/tutorial_remap.html
    //https://stackoverflow.com/questions/46520123/how-do-i-use-opencvs-remap-function
    

    /// Show what you got
    std::string source_window = "Source image";
    std::string warp_window = "rot+scale";


    cv::namedWindow( warp_window, cv::WINDOW_AUTOSIZE );
    cv::imshow( warp_window, warp_dst );

    cv::waitKey(0);

   return ;
}

int main()
{

}